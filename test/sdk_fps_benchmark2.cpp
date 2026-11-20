/**
 * SDK FPS Benchmark v2 — Hypothesis testing
 *
 * Tests:
 *   H1: Camera internally matches/caps RGB to depth rate
 *   H2: Streams don't start/stop cleanly (residual state)
 *   H3: RGB can't start without depth
 *
 * Each test does a FULL disconnect→reconnect to ensure clean state.
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include "3DCamera.hpp"

// ---- Counters ----
static std::atomic<uint64_t> g_depth{0};
static std::atomic<uint64_t> g_rgb{0};

static void depth_cb(cs::IFramePtr frame, void *)
{
    if (frame && !frame->empty())
        g_depth.fetch_add(1, std::memory_order_relaxed);
}

static void rgb_cb(cs::IFramePtr frame, void *)
{
    if (frame && !frame->empty())
        g_rgb.fetch_add(1, std::memory_order_relaxed);
}

static void reset() { g_depth = 0; g_rgb = 0; }

// ---- Connect helper (full fresh connection) ----
struct ConnResult {
    cs::ICameraPtr camera;
    CameraInfo info;
    std::vector<StreamInfo> depth_streams;
    std::vector<StreamInfo> rgb_streams;
    bool ok;
};

static ConnResult fresh_connect()
{
    ConnResult r{};
    r.ok = false;

    cs::ISystemPtr system = cs::getSystemPtr();
    std::vector<CameraInfo> cameras;
    for (int i = 0; i < 20; ++i) {
        system->queryCameras(cameras);
        if (!cameras.empty()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (cameras.empty()) {
        printf("  ERROR: No cameras found!\n");
        return r;
    }

    r.camera = cs::getCameraPtr();
    if (r.camera->connect(cameras[0]) != SUCCESS) {
        printf("  ERROR: connect failed!\n");
        return r;
    }

    r.camera->getInfo(r.info);
    r.camera->getStreamInfos(STREAM_TYPE_DEPTH, r.depth_streams);
    r.camera->getStreamInfos(STREAM_TYPE_RGB, r.rgb_streams);
    r.ok = true;
    return r;
}

static StreamInfo find_stream(const std::vector<StreamInfo>& streams, int format, int pref_w = 0, int pref_h = 0)
{
    StreamInfo best{};
    bool found = false;
    for (auto &s : streams) {
        if (s.format == format) {
            if (!found || (pref_w > 0 && s.width == pref_w && s.height == pref_h)) {
                best = s;
                found = true;
            }
        }
    }
    return best;
}

struct TestResult {
    double elapsed;
    uint64_t depth_frames;
    uint64_t rgb_frames;
    double depth_fps;
    double rgb_fps;
};

static TestResult run_stream_test(cs::ICameraPtr camera,
                                   bool start_depth, StreamInfo depth_si,
                                   bool start_rgb, StreamInfo rgb_si,
                                   int duration_sec, const char* label)
{
    TestResult res{};
    reset();

    ERROR_CODE ret;

    if (start_depth) {
        ret = camera->startStream(STREAM_TYPE_DEPTH, depth_si, depth_cb, nullptr);
        if (ret != SUCCESS) {
            printf("    ERROR: depth startStream failed (ret=%d)\n", ret);
            return res;
        }
    }

    if (start_rgb) {
        ret = camera->startStream(STREAM_TYPE_RGB, rgb_si, rgb_cb, nullptr);
        if (ret != SUCCESS) {
            printf("    ERROR: rgb startStream failed (ret=%d)\n", ret);
            if (start_depth) camera->stopStream(STREAM_TYPE_DEPTH);
            return res;
        }
    }

    auto t0 = std::chrono::steady_clock::now();
    for (int s = 1; s <= duration_sec; ++s) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        uint64_t dc = g_depth.load(), rc = g_rgb.load();

        if (start_depth && start_rgb)
            printf("    [%2d/%ds] Depth: %4lu (%.1f fps) | RGB: %4lu (%.1f fps)\n",
                   s, duration_sec, (unsigned long)dc, dc/elapsed,
                   (unsigned long)rc, rc/elapsed);
        else if (start_depth)
            printf("    [%2d/%ds] Depth: %4lu (%.1f fps)\n",
                   s, duration_sec, (unsigned long)dc, dc/elapsed);
        else
            printf("    [%2d/%ds] RGB: %4lu (%.1f fps)\n",
                   s, duration_sec, (unsigned long)rc, rc/elapsed);
    }

    if (start_depth) camera->stopStream(STREAM_TYPE_DEPTH);
    if (start_rgb)   camera->stopStream(STREAM_TYPE_RGB);

    res.elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    res.depth_frames = g_depth.load();
    res.rgb_frames = g_rgb.load();
    res.depth_fps = res.depth_frames / res.elapsed;
    res.rgb_fps = res.rgb_frames / res.elapsed;
    return res;
}

int main(int argc, char **argv)
{
    int dur = 15;
    if (argc > 1) { dur = std::atoi(argv[1]); if (dur <= 0) dur = 15; }

    printf("========================================================\n");
    printf("  Revopoint SDK Benchmark v2 — Hypothesis Testing\n");
    printf("  Duration: %d seconds per test, full reconnect between\n", dur);
    printf("========================================================\n\n");

    cs::setSdkEnableNetworking(true);
    cs::setEnableNetworking(true);

    // ---- Initial connect to enumerate streams ----
    auto conn = fresh_connect();
    if (!conn.ok) return 1;

    printf("Camera: %s  serial=%s  fw=%s\n\n", conn.info.name, conn.info.serial, conn.info.firmwareVersion);

    printf("Depth streams:\n");
    for (size_t i = 0; i < conn.depth_streams.size(); ++i)
        printf("  [%zu] fmt=%d  %dx%d @ %.1f fps\n", i,
               conn.depth_streams[i].format, conn.depth_streams[i].width,
               conn.depth_streams[i].height, conn.depth_streams[i].fps);
    printf("RGB streams:\n");
    for (size_t i = 0; i < conn.rgb_streams.size(); ++i)
        printf("  [%zu] fmt=%d  %dx%d @ %.1f fps\n", i,
               conn.rgb_streams[i].format, conn.rgb_streams[i].width,
               conn.rgb_streams[i].height, conn.rgb_streams[i].fps);
    printf("\n");

    // Find stream infos
    StreamInfo depth_800  = find_stream(conn.depth_streams, STREAM_FORMAT_Z16, 800, 600);
    StreamInfo depth_1600 = find_stream(conn.depth_streams, STREAM_FORMAT_Z16, 1600, 1200);
    StreamInfo rgb_1600   = find_stream(conn.rgb_streams, STREAM_FORMAT_RGB8, 1600, 1200);

    bool has_depth_800  = (depth_800.width == 800);
    bool has_depth_1600 = (depth_1600.width == 1600);
    bool has_rgb        = (rgb_1600.width == 1600);

    if (!has_rgb) { printf("ERROR: No RGB8 1600x1200 stream!\n"); return 1; }

    conn.camera->disconnect();
    printf("Initial connection closed. Starting tests...\n\n");

    // Results storage
    TestResult results[8] = {};
    const char* labels[8] = {};
    int nTests = 0;

    // ===========================================================
    // TEST A: Depth 800x600 ONLY (baseline depth fps)
    // ===========================================================
    if (has_depth_800) {
        printf("=== TEST A: Depth 800x600 ONLY (fresh connect) ===\n");
        labels[nTests] = "Depth 800x600 only";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        conn = fresh_connect();
        if (conn.ok) {
            results[nTests] = run_stream_test(conn.camera,
                true, depth_800, false, rgb_1600, dur, labels[nTests]);
            printf("  → Depth: %.1f fps\n\n", results[nTests].depth_fps);
            conn.camera->disconnect();
        }
        nTests++;
    }

    // ===========================================================
    // TEST B: RGB8 ONLY — no depth (H3: can RGB run alone?)
    // ===========================================================
    {
        printf("=== TEST B: RGB8 1600x1200 ONLY — no depth (fresh connect) ===\n");
        printf("  H3: Can RGB stream start without depth?\n");
        labels[nTests] = "RGB8-only (no depth)";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        conn = fresh_connect();
        if (conn.ok) {
            results[nTests] = run_stream_test(conn.camera,
                false, depth_800, true, rgb_1600, dur, labels[nTests]);
            printf("  → RGB: %.1f fps  (0 = H3 confirmed: RGB needs depth)\n\n",
                   results[nTests].rgb_fps);
            conn.camera->disconnect();
        }
        nTests++;
    }

    // ===========================================================
    // TEST C: Depth 800x600 + RGB8 (H1: does RGB match depth rate?)
    // ===========================================================
    if (has_depth_800) {
        printf("=== TEST C: Depth 800x600 + RGB8 1600x1200 (fresh connect) ===\n");
        printf("  H1: Does RGB fps match depth fps?\n");
        labels[nTests] = "Depth800 + RGB8";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        conn = fresh_connect();
        if (conn.ok) {
            results[nTests] = run_stream_test(conn.camera,
                true, depth_800, true, rgb_1600, dur, labels[nTests]);
            printf("  → Depth: %.1f fps  RGB: %.1f fps\n\n",
                   results[nTests].depth_fps, results[nTests].rgb_fps);
            conn.camera->disconnect();
        }
        nTests++;
    }

    // ===========================================================
    // TEST D: Depth 1600x1200 + RGB8 (H1: different depth res = different cap?)
    // ===========================================================
    if (has_depth_1600) {
        printf("=== TEST D: Depth 1600x1200 + RGB8 1600x1200 (fresh connect) ===\n");
        printf("  H1: Does RGB fps change with depth resolution?\n");
        labels[nTests] = "Depth1600 + RGB8";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        conn = fresh_connect();
        if (conn.ok) {
            results[nTests] = run_stream_test(conn.camera,
                true, depth_1600, true, rgb_1600, dur, labels[nTests]);
            printf("  → Depth: %.1f fps  RGB: %.1f fps\n\n",
                   results[nTests].depth_fps, results[nTests].rgb_fps);
            conn.camera->disconnect();
        }
        nTests++;
    }

    // ===========================================================
    // TEST E: Depth 800x600 ONLY (second run — H2: clean restart?)
    // ===========================================================
    if (has_depth_800) {
        printf("=== TEST E: Depth 800x600 ONLY — second run (fresh connect) ===\n");
        printf("  H2: Same fps as TEST A? (clean restart verification)\n");
        labels[nTests] = "Depth800 only (2nd)";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        conn = fresh_connect();
        if (conn.ok) {
            results[nTests] = run_stream_test(conn.camera,
                true, depth_800, false, rgb_1600, dur, labels[nTests]);
            printf("  → Depth: %.1f fps  (compare TEST A)\n\n", results[nTests].depth_fps);
            conn.camera->disconnect();
        }
        nTests++;
    }

    // ===========================================================
    // TEST F: Depth 800x600 + RGB — start depth first, wait 5s, then add RGB
    // H2: Does delayed start affect fps?
    // ===========================================================
    if (has_depth_800) {
        printf("=== TEST F: Staggered start — Depth first, then RGB after 5s ===\n");
        printf("  H2: Does start order / warm-up affect RGB fps?\n");
        labels[nTests] = "Staggered start";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        conn = fresh_connect();
        if (conn.ok) {
            reset();
            ERROR_CODE ret;

            // Start depth only
            ret = conn.camera->startStream(STREAM_TYPE_DEPTH, depth_800, depth_cb, nullptr);
            if (ret == SUCCESS) {
                printf("    Depth started. Waiting 5s for warm-up...\n");
                for (int s = 1; s <= 5; ++s) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    printf("    [warm %d/5s] Depth: %lu\n", s, (unsigned long)g_depth.load());
                }

                // Now add RGB
                reset(); // reset counters for the measured period
                ret = conn.camera->startStream(STREAM_TYPE_RGB, rgb_1600, rgb_cb, nullptr);
                if (ret == SUCCESS) {
                    printf("    RGB started. Measuring %ds...\n", dur);
                    auto t0 = std::chrono::steady_clock::now();
                    for (int s = 1; s <= dur; ++s) {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        double elapsed = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - t0).count();
                        printf("    [%2d/%ds] Depth: %4lu (%.1f fps) | RGB: %4lu (%.1f fps)\n",
                               s, dur,
                               (unsigned long)g_depth.load(), g_depth.load()/elapsed,
                               (unsigned long)g_rgb.load(), g_rgb.load()/elapsed);
                    }
                    conn.camera->stopStream(STREAM_TYPE_RGB);

                    double elapsed = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();
                    results[nTests].elapsed = elapsed;
                    results[nTests].depth_frames = g_depth.load();
                    results[nTests].rgb_frames = g_rgb.load();
                    results[nTests].depth_fps = g_depth.load() / elapsed;
                    results[nTests].rgb_fps = g_rgb.load() / elapsed;
                } else {
                    printf("    ERROR: RGB startStream failed (ret=%d)\n", ret);
                }
                conn.camera->stopStream(STREAM_TYPE_DEPTH);
            } else {
                printf("    ERROR: depth startStream failed (ret=%d)\n", ret);
            }
            printf("  → Depth: %.1f fps  RGB: %.1f fps\n\n",
                   results[nTests].depth_fps, results[nTests].rgb_fps);
            conn.camera->disconnect();
        }
        nTests++;
    }

    // ===========================================================
    // TEST G: Depth 800x600 + RGB — NO disconnect since TEST F
    // Stream stop → immediate re-start (H2: does stop/start leak state?)
    // ===========================================================
    if (has_depth_800) {
        printf("=== TEST G: Immediate re-stream (NO disconnect, same session) ===\n");
        printf("  H2: Stop+restart without disconnect — clean?\n");
        labels[nTests] = "Re-stream (no disc)";
        // Don't disconnect — reconnect
        conn = fresh_connect();
        if (conn.ok) {
            // First run: start+stop
            reset();
            conn.camera->startStream(STREAM_TYPE_DEPTH, depth_800, depth_cb, nullptr);
            conn.camera->startStream(STREAM_TYPE_RGB, rgb_1600, rgb_cb, nullptr);
            printf("    First session: streaming 5s then stopping...\n");
            std::this_thread::sleep_for(std::chrono::seconds(5));
            printf("    After 5s: Depth=%lu RGB=%lu\n",
                   (unsigned long)g_depth.load(), (unsigned long)g_rgb.load());
            conn.camera->stopStream(STREAM_TYPE_RGB);
            conn.camera->stopStream(STREAM_TYPE_DEPTH);

            // Brief pause then restart WITHOUT disconnect
            printf("    Stopped. Pausing 2s then restarting (no disconnect)...\n");
            std::this_thread::sleep_for(std::chrono::seconds(2));

            reset();
            conn.camera->startStream(STREAM_TYPE_DEPTH, depth_800, depth_cb, nullptr);
            conn.camera->startStream(STREAM_TYPE_RGB, rgb_1600, rgb_cb, nullptr);

            auto t0 = std::chrono::steady_clock::now();
            for (int s = 1; s <= dur; ++s) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();
                printf("    [%2d/%ds] Depth: %4lu (%.1f fps) | RGB: %4lu (%.1f fps)\n",
                       s, dur,
                       (unsigned long)g_depth.load(), g_depth.load()/elapsed,
                       (unsigned long)g_rgb.load(), g_rgb.load()/elapsed);
            }

            conn.camera->stopStream(STREAM_TYPE_RGB);
            conn.camera->stopStream(STREAM_TYPE_DEPTH);

            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            results[nTests].elapsed = elapsed;
            results[nTests].depth_frames = g_depth.load();
            results[nTests].rgb_frames = g_rgb.load();
            results[nTests].depth_fps = g_depth.load() / elapsed;
            results[nTests].rgb_fps = g_rgb.load() / elapsed;

            printf("  → Depth: %.1f fps  RGB: %.1f fps\n\n",
                   results[nTests].depth_fps, results[nTests].rgb_fps);
            conn.camera->disconnect();
        }
        nTests++;
    }

    // ===========================================================
    // SUMMARY
    // ===========================================================
    printf("\n");
    printf("==================== SUMMARY ====================\n");
    printf("%-25s  %10s  %10s\n", "Test", "Depth fps", "RGB fps");
    printf("-------------------------------------------------\n");
    for (int i = 0; i < nTests; ++i) {
        if (labels[i]) {
            printf("%-25s  %10.1f  %10.1f\n", labels[i],
                   results[i].depth_fps, results[i].rgb_fps);
        }
    }
    printf("=================================================\n\n");

    printf("HYPOTHESIS ANALYSIS:\n");
    printf("  H1 (RGB capped to depth): Compare C vs D.\n");
    printf("     If RGB fps ≈ depth fps in both → CONFIRMED\n");
    printf("     If RGB fps stays same regardless of depth res → REJECTED\n\n");
    printf("  H2 (dirty start/stop): Compare A vs E (depth-only repeat),\n");
    printf("     and C vs G (with/without disconnect).\n");
    printf("     Significant difference → CONFIRMED\n\n");
    printf("  H3 (RGB needs depth): See TEST B.\n");
    printf("     0 fps → CONFIRMED, >0 fps → REJECTED\n\n");

    return 0;
}

/*
 * BUILD:
 *   SDK=/home/ubuntu/ros2_ws/src/revopoint_camera/3DCameraSDK-v3.2.229.20251110
 *   g++ -std=c++17 -O2 \
 *     -I $SDK/inc/3dcamera \
 *     test/sdk_fps_benchmark2.cpp \
 *     -L $SDK/lib/3dcamera/linux/x64 \
 *     -l3DCamera -lpthread \
 *     -Wl,-rpath,$SDK/lib/3dcamera/linux/x64 \
 *     -o sdk_fps_benchmark2
 *
 * RUN:
 *   ./sdk_fps_benchmark2 [duration_seconds]
 */
