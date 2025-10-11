#include "led-matrix.h"
#include "graphics.h"

#include <unistd.h>
#include <ctime>
#include <vector>
#include <cmath>
#include <sys/time.h>
#include <algorithm>
#include <stdlib.h>
#include <string>

using namespace std;
using rgb_matrix::GPIO;
using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

#define SCREEN_SIZE 32
#define M_PI 3.14159265358979323846

// Wave rendering parameters
static const float VISIBILITY_CONST = 1024.0f;   // Keeps waves visible at ~32 px (attenuation ~0.5)
static const float WAVE_SIGMA = 2.6f;            // Gaussian width of the wavefront for single-front Gabor (wider)
static const float WAVELENGTH = 4.0f;            // Spatial wavelength in pixels
static const float SECOND_WAVE_OFFSET = WAVELENGTH * 0.8f;  // Unused when single front, kept for tuning
static const float AMPLITUDE_GAIN = 1.8f;        // Rebalanced for thicker fronts without clipping

// Removed DoG parameters in favor of a single gated Gabor pulse

// Edge reflection parameters (image sources)
static const int ENABLE_REFLECTIONS = 0;
static const float REFLECT_GAIN_EDGE = 0.6f;     // Reflection coefficient for single-edge images
static const float REFLECT_GAIN_CORNER = 0.36f;  // Typically edge^2 for double reflections at corners
static const int REFLECT_PHASE_FLIP = 0;         // If 1: phase inversion on odd reflections (edges)

// Simulation parameters
static const float WAVE_SPEED = 5.0f;            // pixels per second
static const int MAX_ACTIVE_DROPLETS = 3;        // limit active droplets
static const float MIN_DROP_INTERVAL = 2.5f;     // seconds between spawns
static const float DROP_INTERVAL_JITTER = 1.0f;  // additional random seconds [0, DROP_INTERVAL_JITTER]
static const float VISIBILITY_CUTOFF = 0.01f;    // amplitude threshold to cull a droplet

// Smoothing / performance
static const float TONE_MAP_GAIN = 1.0f;         // tanh limiter gain for amplitude
static const float EMA_ALPHA = 0.85f;            // temporal smoothing factor per pixel
static const int REFLECT_ENABLE_CORNERS = 0;     // disable corner reflections to save CPU
static const int EDGE_REFLECT_MAX_DISTANCE = 8;  // only evaluate edge images near edges
static const int MAX_REFLECTION_ORDER = 2;       // allow reflected waves to reflect again (up to 2)

// Hue mapping mode: 0 = original linear angle mapping (seam), 1 = continuous sine band
static const int USE_CONTINUOUS_HUE = 1;

// Per-pixel EMA buffer
static std::vector<float> g_ema_buffer;

struct Point {
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}
};

struct Droplet {
    Point center;
    double start_time;
};

vector<Droplet> droplets;
double last_drop_time = 0.0;

rgb_matrix::Color getColorFromHSV(float h, float s, float v) {
    h = fmod(h, 1.0f) * 6.0f;
    int i = floor(h);
    float f = h - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    float r, g, b;
    switch (i % 6) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
    return rgb_matrix::Color(static_cast<uint8_t>(r * 255), static_cast<uint8_t>(g * 255), static_cast<uint8_t>(b * 255));
}

int main(int argc, char *argv[]) {
    GPIO io;
    if (!io.Init())
        return 1;

    RGBMatrix *matrix = new RGBMatrix(&io, SCREEN_SIZE, 1);
    rgb_matrix::FrameCanvas *offscreen = matrix->CreateFrameCanvas();

    // Load font for time display
    rgb_matrix::Font time_font;
    if (!time_font.LoadFont("pongnumberfont.bdf"))
    {
        fprintf(stderr, "Failed to load font: pongnumberfont.bdf\n");
    }

    srand(time(0));

    time_t t = time(0);
    time_t startTime = time(0);

    int maxtime = 0;
    if (argc > 1) {
        string test(argv[1]);
        maxtime = std::stoi(test);
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    double current_time = tv.tv_sec + tv.tv_usec / 1e6;
    last_drop_time = current_time;

    // Init EMA buffer
    g_ema_buffer.assign(SCREEN_SIZE * SCREEN_SIZE, 0.0f);

    bool cont = true;
    while (cont) {
        gettimeofday(&tv, NULL);
        current_time = tv.tv_sec + tv.tv_usec / 1e6;

        // Add new droplet slowly, cap active droplets
        if ((int)droplets.size() < MAX_ACTIVE_DROPLETS) {
            float interval = MIN_DROP_INTERVAL + ((rand() % 1000) / 1000.0f) * DROP_INTERVAL_JITTER;
            if (current_time - last_drop_time > interval) {
                Droplet d;
                d.center = Point(rand() % SCREEN_SIZE, rand() % SCREEN_SIZE);
                d.start_time = current_time;
                droplets.push_back(d);
                last_drop_time = current_time;
            }
        }

        // Remove old or invisible droplets (age or amplitude-based cull)
        vector<Droplet>::iterator it = droplets.begin();
        while (it != droplets.end()) {
            double age = current_time - it->start_time;
            bool remove = false;
            if (age > 12.0) {
                remove = true;
            } else {
                // Peak amplitude at the wavefront for direct path at center pixel of the front
                // envelope is 1 at phase=0; attenuation at radius r ~ VISIBILITY_CONST/(VISIBILITY_CONST + r^2)
                float radius = static_cast<float>(age * WAVE_SPEED);
                // Consider worst-case: furthest diagonal distance to keep reflections meaningful
                float max_r = radius;
                float attenuation = VISIBILITY_CONST / (VISIBILITY_CONST + max_r * max_r);
                float peak_amplitude = attenuation * AMPLITUDE_GAIN * exp(-age / 5.0);
                if (peak_amplitude < VISIBILITY_CUTOFF) {
                    remove = true;
                }
            }
            if (remove) {
                it = droplets.erase(it);
            } else {
                ++it;
            }
        }

        offscreen->Fill(0, 0, 0);

        // Draw all droplets
        for (int py = 0; py < SCREEN_SIZE; ++py) {
            for (int px = 0; px < SCREEN_SIZE; ++px) {
                float amplitude_total = 0.0f;

                for (const auto& drop : droplets) {
                    double age = current_time - drop.start_time;
                    float speed = WAVE_SPEED; // pixels per second
                    float brightness = exp(-age / 5.0); // fade out over time

                    auto accumulate_from_source = [&](float sx, float sy, float gain, bool phase_flip)
                    {
                        float dx = px - sx + 0.5f;
                        float dy = py - sy + 0.5f;
                        float dist = hypot(dx, dy);
                        float attenuation = VISIBILITY_CONST / (VISIBILITY_CONST + dist * dist);

                        float amplitude_sum_local = 0.0f;
                        float radius = age * speed;
                        float phase = dist - radius;
                        // Smooth gate around the front to avoid popping: smoothstep over [-g, 0]
                        const float gate_width = WAVE_SIGMA; // tie gate width to sigma
                        float gate = 0.0f;
                        if (phase <= 0.0f)
                        {
                            if (phase >= -gate_width)
                            {
                                float x = (phase + gate_width) / gate_width; // 0..1
                                gate = x * x * (3.0f - 2.0f * x); // smoothstep
                            }
                            else
                            {
                                gate = 1.0f;
                            }
                            float envelope = exp(-0.5f * (phase * phase) / (WAVE_SIGMA * WAVE_SIGMA));
                            float carrier = cos(2.0f * M_PI * phase / WAVELENGTH);
                            if (phase_flip)
                            {
                                carrier = -carrier;
                            }
                            amplitude_sum_local = envelope * carrier * gate;
                        }

                        float amplitude_local = amplitude_sum_local * brightness * attenuation * AMPLITUDE_GAIN * gain;
                        if (fabs(amplitude_local) > 1e-5f)
                        {
                            amplitude_total += amplitude_local;
                        }
                    };

                    // Direct source
                    accumulate_from_source(static_cast<float>(drop.center.x), static_cast<float>(drop.center.y), 1.0f, false);

                    if (ENABLE_REFLECTIONS)
                    {
                        // Higher-order reflections via mirrored tiling up to MAX_REFLECTION_ORDER
                        // Base source position
                        float x0 = static_cast<float>(drop.center.x);
                        float y0 = static_cast<float>(drop.center.y);

                        bool flip_edge = (REFLECT_PHASE_FLIP != 0);

                        for (int rx = -MAX_REFLECTION_ORDER; rx <= MAX_REFLECTION_ORDER; ++rx)
                        {
                            for (int ry = -MAX_REFLECTION_ORDER; ry <= MAX_REFLECTION_ORDER; ++ry)
                            {
                                if (rx == 0 && ry == 0)
                                {
                                    continue; // direct already added
                                }

                                // Skip far tiles to save work
                                if (abs(rx) + abs(ry) > MAX_REFLECTION_ORDER)
                                {
                                    continue;
                                }

                                // Compute mirrored coordinate using odd-even reflection across boundaries
                                // Reflect across lines at -0.5 and SCREEN_SIZE-0.5 using 2*(SCREEN_SIZE) periodicity with reflection
                                auto reflect_coord = [](float c, int k) -> float
                                {
                                    float period = 2.0f * SCREEN_SIZE;
                                    float base = c;
                                    float shift = k * period;
                                    // Every odd k flips around SCREEN_SIZE - 1
                                    if ((k & 1) == 0)
                                    {
                                        return base + shift;
                                    }
                                    else
                                    {
                                        return (SCREEN_SIZE - 1.0f) - base + shift;
                                    }
                                };

                                float sx = reflect_coord(x0, rx);
                                float sy = reflect_coord(y0, ry);

                                // Reflection gain decays with order; edges get REFLECT_GAIN_EDGE, corners approx squared
                                int order = abs(rx) + abs(ry);
                                float gain = 1.0f;
                                bool phase_flip = false;
                                if (order == 1)
                                {
                                    gain = REFLECT_GAIN_EDGE;
                                    phase_flip = flip_edge;
                                }
                                else if (order == 2)
                                {
                                    // Two reflections: roughly square of edge gain
                                    gain = REFLECT_GAIN_CORNER;
                                    phase_flip = flip_edge; // optionally flip if odd number of edge reflections
                                }

                                // Proximity gating: only evaluate tiles relevant to current pixel region
                                bool near_x_edge = (px < EDGE_REFLECT_MAX_DISTANCE) || ((SCREEN_SIZE - 1 - px) < EDGE_REFLECT_MAX_DISTANCE);
                                bool near_y_edge = (py < EDGE_REFLECT_MAX_DISTANCE) || ((SCREEN_SIZE - 1 - py) < EDGE_REFLECT_MAX_DISTANCE);
                                if (order == 1)
                                {
                                    // Only along the corresponding axis
                                    if ((rx != 0 && !near_x_edge) || (ry != 0 && !near_y_edge))
                                    {
                                        continue;
                                    }
                                }
                                else if (order == 2)
                                {
                                    if (!(near_x_edge && near_y_edge))
                                    {
                                        continue;
                                    }
                                }

                                accumulate_from_source(sx, sy, gain, phase_flip);
                            }
                        }
                    }
                }

                // Map the summed signed amplitude to a single hue based on pixel angle from screen center
                float cx = (SCREEN_SIZE * 0.5f);
                float cy = (SCREEN_SIZE * 0.5f);
                float gdx = px - cx + 0.5f;
                float gdy = py - cy + 0.5f;
                float angle_global = atan2(gdy, gdx) / (2.0f * M_PI) + 0.5f;
                float hue_global;
                if (USE_CONTINUOUS_HUE)
                {
                    hue_global = 0.5f + 0.2f * sinf(2.0f * M_PI * angle_global);
                }
                else
                {
                    hue_global = 0.7f - angle_global * 0.4f;
                }
                rgb_matrix::Color col_global = getColorFromHSV(hue_global, 1.0f, 1.0f);

                // Tone map amplitude with tanh limiter and apply temporal smoothing (EMA)
                float a = tanhf(TONE_MAP_GAIN * amplitude_total);
                int buf_idx = py * SCREEN_SIZE + px;
                float ema_prev = g_ema_buffer[buf_idx];
                float ema_cur = EMA_ALPHA * ema_prev + (1.0f - EMA_ALPHA) * a;
                g_ema_buffer[buf_idx] = ema_cur;

                float out_r_f = col_global.r * ema_cur;
                float out_g_f = col_global.g * ema_cur;
                float out_b_f = col_global.b * ema_cur;

                uint8_t r = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, out_r_f)));
                uint8_t g = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, out_g_f)));
                uint8_t b = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, out_b_f)));

                offscreen->SetPixel(px, py, r, g, b);
            }
        }

        // Draw current time (HH:MM:SS) top-right using pongnumberfont
        {
            time_t now = time(NULL);
            struct tm tm_now;
            localtime_r(&now, &tm_now);
            char time_str[9];
            strftime(time_str, sizeof(time_str), "%H:%M:%S", &tm_now);

            // Animate time color through the same hue band as wavefronts (0.7 -> 0.3)
            float cycle_period = 12.0f; // seconds per full cycle
            float tnorm = fmodf(static_cast<float>(current_time), cycle_period) / cycle_period; // 0..1
            float hue_time = 0.7f - tnorm * 0.4f; // match wavefront hue range
            rgb_matrix::Color time_color = getColorFromHSV(hue_time, 1.0f, 0.5f); // 50% brightness

            int letter_spacing = 0;
            int baseline = time_font.baseline();

            // Measure width off-screen (y negative to avoid visible draw while measuring)
            int measure_start_x = 0;
            int end_x = rgb_matrix::DrawText(offscreen, time_font, measure_start_x, -100, time_color, NULL, time_str, letter_spacing);
            int text_width = end_x - measure_start_x;
            int start_x = SCREEN_SIZE - text_width - 3;
            if (start_x < 0)
            {
                start_x = 0; // clamp if too wide
            }

            rgb_matrix::DrawText(offscreen, time_font, start_x, baseline, time_color, NULL, time_str, letter_spacing);
        }

        // Swap to screen on vsync for tear-free update
        offscreen = matrix->SwapOnVSync(offscreen);


        t = time(0);
        if (maxtime > 0) {
            if (difftime(t, startTime) > maxtime) {
                cont = false;
                printf("stopping now\n");
            }
        }
    }

    matrix->Clear();
    delete matrix;
    return 0;
}
