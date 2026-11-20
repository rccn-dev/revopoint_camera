/**
 * SDK-only FPS benchmark for Revopoint camera.
 * Tests raw frame delivery rate WITHOUT any ROS overhead.
 *
 * Build & run instructions at the bottom of this file.
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

// ---- Global counters ----
static std::atomic<uint64_t> depth_count{0};
static std::atomic<uint64_t> rgb_count{0};
static std::atomic<int64_t>  depth_cb_total_ns{0};
static std::atomic<int64_t>  rgb_cb_total_ns{0};

// ---- MJPG counters ----
static std::atomic<uint64_t> mjpg_count{0};
static std::atomic<uint64_t> mjpg_total_bytes{0};
static std::atomic<int>      mjpg_last_size{0};

// ---- RGB-only / MJPG-only counters ----
static std::atomic<uint64_t> rgb_only_count{0};
static std::atomic<uint64_t> mjpg_only_count{0};
static std::atomic<uint64_t> mjpg_only_total_bytes{0};
static std::atomic<int>      mjpg_only_last_size{0};

static void rgb_cb_only(cs::IFramePtr frame, void *)
{
    if (frame && !frame->empty())
        rgb_only_count.fetch_add(1, std::memory_order_relaxed);
}

static void mjpg_cb_only(cs::IFramePtr frame, void *)
{
    if (!frame || frame->empty()) return;
    int sz = frame->getSize();
    mjpg_only_count.fetch_add(1, std::memory_order_relaxed);
    mjpg_only_total_bytes.fetch_add(sz, std::memory_order_relaxed);
    mjpg_only_last_size.store(sz, std::memory_order_relaxed);
}

// ---- TEST 1: Minimal callbacks (just count) ----
static void depth_cb_minimal(cs::IFramePtr frame, void *)
{
    auto t0 = std::chrono::steady_clock::now();
    if (frame && !frame->empty())
        depth_count.fetch_add(1, std::memory_order_relaxed);
    auto t1 = std::chrono::steady_clock::now();
    depth_cb_total_ns.fetch_add(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count(),
        std::memory_order_relaxed);
}

static void rgb_cb_minimal(cs::IFramePtr frame, void *)
{
    auto t0 = std::chrono::steady_clock::now();
    if (frame && !frame->empty())
        rgb_count.fetch_add(1, std::memory_order_relaxed);
    auto t1 = std::chrono::steady_clock::now();
    rgb_cb_total_ns.fetch_add(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count(),
        std::memory_order_relaxed);
}

// ---- TEST 4: MJPG callback (compressed stream) ----
static void rgb_cb_mjpg(cs::IFramePtr frame, void *)
{
    if (!frame || frame->empty()) return;
    int sz = frame->getSize();
    mjpg_count.fetch_add(1, std::memory_order_relaxed);
    mjpg_total_bytes.fetch_add(sz, std::memory_order_relaxed);
    mjpg_last_size.store(sz, std::memory_order_relaxed);
}

// ---- TEST 2: Heavy callbacks (memcpy + pixel iteration) ----
static std::atomic<uint64_t> depth_heavy_count{0};
static std::atomic<uint64_t> rgb_heavy_count{0};

static void depth_cb_heavy(cs::IFramePtr frame, void *)
{
    if (!frame || frame->empty()) return;
    int sz = frame->getSize();
    // Allocate + copy (simulates ROS Image message creation)
    std::vector<uint8_t> buf(sz);
    std::memcpy(buf.data(), frame->getData(FRAME_DATA_FORMAT_Z16), sz);
    // Simulate per-pixel scale conversion (1.92M pixels at 1600x1200)
    volatile uint64_t sum = 0;
    const uint16_t *p = reinterpret_cast<const uint16_t *>(buf.data());
    for (int i = 0; i < sz / 2; ++i) {
        sum += p[i];
    }
    (void)sum;
    depth_heavy_count.fetch_add(1, std::memory_order_relaxed);
}

static void rgb_cb_heavy(cs::IFramePtr frame, void *)
{
    if (!frame || frame->empty()) return;
    int sz = frame->getSize();
    // Allocate + copy (simulates ROS Image message creation)
    std::vector<uint8_t> buf(sz);
    std::memcpy(buf.data(), frame->getData(), sz);
    rgb_heavy_count.fetch_add(1, std::memory_order_relaxed);
}

// ---- Helpers ----
static void reset_counters()
{
    depth_count = 0;
    rgb_count = 0;
    depth_cb_total_ns = 0;
    rgb_cb_total_ns = 0;
    depth_heavy_count = 0;
    rgb_heavy_count = 0;
    mjpg_count = 0;
    mjpg_total_bytes = 0;
    mjpg_last_size = 0;
    rgb_only_count = 0;
    mjpg_only_count = 0;
    mjpg_only_total_bytes = 0;
    mjpg_only_last_size = 0;
}

static void print_live(int sec, int total, double elapsed)
{
    uint64_t dc = depth_count.load();
    uint64_t rc = rgb_count.load();
    printf("  [%2d/%d s]  Depth: %4lu frames (%.1f fps)  |  RGB: %4lu frames (%.1f fps)\n",
           sec, total, (unsigned long)dc, dc / elapsed,
           (unsigned long)rc, rc / elapsed);
}

static void print_live_heavy(int sec, int total, double elapsed)
{
    uint64_t dc = depth_heavy_count.load();
    uint64_t rc = rgb_heavy_count.load();
    printf("  [%2d/%d s]  Depth: %4lu frames (%.1f fps)  |  RGB: %4lu frames (%.1f fps)\n",
           sec, total, (unsigned long)dc, dc / elapsed,
           (unsigned long)rc, rc / elapsed);
}

int main(int argc, char **argv)
{
    int duration_sec = 15;
    if (argc > 1) {
        duration_sec = std::atoi(argv[1]);
        if (duration_sec <= 0) duration_sec = 15;
    }

    printf("=== Revopoint SDK FPS Benchmark (no ROS) ===\n");
    printf("Test duration: %d seconds per test\n\n", duration_sec);

    // ---- Enable networking for Ethernet cameras ----
    cs::setSdkEnableNetworking(true);
    cs::setEnableNetworking(true);
    printf("Network discovery enabled.\n");

    // ---- Discover cameras (retry with timeout for network cameras) ----
    cs::ISystemPtr system = cs::getSystemPtr();
    std::vector<CameraInfo> cameras;
    ERROR_CODE ret;

    printf("Searching for cameras (up to 10s)...\n");
    for (int i = 0; i < 20; ++i) {
        ret = system->queryCameras(cameras);
        if (ret == SUCCESS && !cameras.empty()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (cameras.empty()) {
        printf("ERROR: No cameras found after 10s (ret=%d)!\n", ret);
        printf("Check: is camera at 192.168.3.99 reachable? (ping it)\n");
        return 1;
    }
    printf("Found %zu camera(s)\n", cameras.size());
    for (auto &c : cameras) {
        printf("  - %s (serial: %s)\n", c.name, c.serial);
    }

    // ---- Connect ----
    cs::ICameraPtr camera = cs::getCameraPtr();
    ret = camera->connect(cameras[0]);
    if (ret != SUCCESS) {
        printf("ERROR: connect failed (ret=%d)\n", ret);
        return 1;
    }

    CameraInfo info{};
    camera->getInfo(info);
    printf("\nConnected: %s  serial=%s  fw=%s\n", info.name, info.serial, info.firmwareVersion);

    // ---- List available streams ----
    std::vector<StreamInfo> depth_streams, rgb_streams;
    camera->getStreamInfos(STREAM_TYPE_DEPTH, depth_streams);
    camera->getStreamInfos(STREAM_TYPE_RGB, rgb_streams);

    printf("\nDepth streams:\n");
    for (size_t i = 0; i < depth_streams.size(); ++i)
        printf("  [%zu] format=%d  %dx%d @ %.1f fps\n",
               i, depth_streams[i].format, depth_streams[i].width,
               depth_streams[i].height, depth_streams[i].fps);

    printf("RGB streams:\n");
    for (size_t i = 0; i < rgb_streams.size(); ++i)
        printf("  [%zu] format=%d  %dx%d @ %.1f fps\n",
               i, rgb_streams[i].format, rgb_streams[i].width,
               rgb_streams[i].height, rgb_streams[i].fps);

    // ---- Select streams (same logic as the ROS node) ----
    StreamInfo depth_si{};
    bool found_depth = false;
    for (auto &s : depth_streams) {
        if (s.format == STREAM_FORMAT_Z16) {
            depth_si = s;
            found_depth = true;
            break;
        }
    }
    if (!found_depth) {
        printf("ERROR: No Z16 depth stream!\n");
        camera->disconnect();
        return 1;
    }

    StreamInfo rgb_si{};
    bool found_rgb = false;
    for (auto &s : rgb_streams) {
        if (s.format == STREAM_FORMAT_RGB8) {
            rgb_si = s;
            found_rgb = true;
            break;
        }
    }
    if (!found_rgb) {
        printf("WARNING: No RGB8 stream, will test depth only.\n");
    }

    // ---- Also find MJPG stream for TEST 4 ----
    StreamInfo mjpg_si{};
    bool found_mjpg = false;
    for (auto &s : rgb_streams) {
        if (s.format == STREAM_FORMAT_MJPG) {
            mjpg_si = s;
            found_mjpg = true;
            break;
        }
    }
    if (!found_mjpg) {
        printf("WARNING: No MJPG stream found, will skip MJPG test.\n");
    }

    printf("\nSelected depth: %dx%d @ %.1f fps (format=%d)\n",
           depth_si.width, depth_si.height, depth_si.fps, depth_si.format);
    if (found_rgb)
        printf("Selected RGB:   %dx%d @ %.1f fps (format=%d)\n",
               rgb_si.width, rgb_si.height, rgb_si.fps, rgb_si.format);
    if (found_mjpg)
        printf("Selected MJPG:  %dx%d @ %.1f fps (format=%d)\n",
               mjpg_si.width, mjpg_si.height, mjpg_si.fps, mjpg_si.format);

    double expected_depth_fps = depth_si.fps > 0 ? depth_si.fps : 6.0;
    double expected_rgb_fps = found_rgb ? (rgb_si.fps > 0 ? rgb_si.fps : 20.0) : 0.0;

    // =========================================================
    // TEST 1: Minimal callback
    // =========================================================
    printf("\n========== TEST 1: Minimal callback (count only) ==========\n");
    reset_counters();

    ret = camera->startStream(STREAM_TYPE_DEPTH, depth_si, depth_cb_minimal, nullptr);
    if (ret != SUCCESS) {
        printf("ERROR: depth startStream failed (ret=%d)\n", ret);
        camera->disconnect();
        return 1;
    }
    if (found_rgb) {
        ret = camera->startStream(STREAM_TYPE_RGB, rgb_si, rgb_cb_minimal, nullptr);
        if (ret != SUCCESS) {
            printf("ERROR: rgb startStream failed (ret=%d)\n", ret);
            camera->stopStream(STREAM_TYPE_DEPTH);
            camera->disconnect();
            return 1;
        }
    }

    auto t_start = std::chrono::steady_clock::now();
    for (int s = 1; s <= duration_sec; ++s) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        print_live(s, duration_sec, elapsed);
    }

    camera->stopStream(STREAM_TYPE_DEPTH);
    if (found_rgb) camera->stopStream(STREAM_TYPE_RGB);

    double t1_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    uint64_t t1_depth = depth_count.load();
    uint64_t t1_rgb = rgb_count.load();

    printf("\n  RESULT (minimal):\n");
    printf("    Depth: %lu frames / %.1fs = %.2f fps (expected ~%.0f)\n",
           (unsigned long)t1_depth, t1_elapsed, t1_depth / t1_elapsed, expected_depth_fps);
    if (t1_depth > 0)
        printf("    Depth avg callback: %.1f µs\n",
               depth_cb_total_ns.load() / (double)t1_depth / 1000.0);
    if (found_rgb) {
        printf("    RGB:   %lu frames / %.1fs = %.2f fps (expected ~%.0f)\n",
               (unsigned long)t1_rgb, t1_elapsed, t1_rgb / t1_elapsed, expected_rgb_fps);
        if (t1_rgb > 0)
            printf("    RGB avg callback: %.1f µs\n",
                   rgb_cb_total_ns.load() / (double)t1_rgb / 1000.0);
    }

    // Brief rest
    printf("\n  (pause 3s between tests)\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // =========================================================
    // TEST 2: Heavy callback (simulates old ROS node work)
    // =========================================================
    printf("\n========== TEST 2: Heavy callback (memcpy + pixel scan) ==========\n");
    reset_counters();

    ret = camera->startStream(STREAM_TYPE_DEPTH, depth_si, depth_cb_heavy, nullptr);
    if (ret != SUCCESS) {
        printf("ERROR: depth startStream failed (ret=%d)\n", ret);
        camera->disconnect();
        return 1;
    }
    if (found_rgb) {
        ret = camera->startStream(STREAM_TYPE_RGB, rgb_si, rgb_cb_heavy, nullptr);
        if (ret != SUCCESS) {
            printf("ERROR: rgb startStream failed (ret=%d)\n", ret);
            camera->stopStream(STREAM_TYPE_DEPTH);
            camera->disconnect();
            return 1;
        }
    }

    t_start = std::chrono::steady_clock::now();
    for (int s = 1; s <= duration_sec; ++s) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        print_live_heavy(s, duration_sec, elapsed);
    }

    camera->stopStream(STREAM_TYPE_DEPTH);
    if (found_rgb) camera->stopStream(STREAM_TYPE_RGB);

    double t2_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    uint64_t t2_depth = depth_heavy_count.load();
    uint64_t t2_rgb = rgb_heavy_count.load();

    printf("\n  RESULT (heavy):\n");
    printf("    Depth: %lu frames / %.1fs = %.2f fps\n",
           (unsigned long)t2_depth, t2_elapsed, t2_depth / t2_elapsed);
    if (found_rgb)
        printf("    RGB:   %lu frames / %.1fs = %.2f fps\n",
               (unsigned long)t2_rgb, t2_elapsed, t2_rgb / t2_elapsed);

    printf("\n  (pause 3s between tests)\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // =========================================================
    // TEST 3: Polling mode
    // =========================================================
    printf("\n========== TEST 3: Polling mode (getFrame) ==========\n");

    ret = camera->startStream(STREAM_TYPE_DEPTH, depth_si);
    if (ret != SUCCESS) {
        printf("ERROR: depth startStream (polling) failed (ret=%d)\n", ret);
        camera->disconnect();
        return 1;
    }
    if (found_rgb) {
        ret = camera->startStream(STREAM_TYPE_RGB, rgb_si);
        if (ret != SUCCESS) {
            printf("ERROR: rgb startStream (polling) failed (ret=%d)\n", ret);
            camera->stopStream(STREAM_TYPE_DEPTH);
            camera->disconnect();
            return 1;
        }
    }

    uint64_t poll_depth = 0, poll_rgb = 0;
    t_start = std::chrono::steady_clock::now();
    auto t_last_print = t_start;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - t_start).count();
        if (elapsed >= duration_sec) break;

        cs::IFramePtr frame;
        // Use short timeout to keep polling both streams
        if (camera->getFrame(STREAM_TYPE_DEPTH, frame, 50) == SUCCESS && frame && !frame->empty()) {
            poll_depth++;
        }
        if (found_rgb) {
            if (camera->getFrame(STREAM_TYPE_RGB, frame, 50) == SUCCESS && frame && !frame->empty()) {
                poll_rgb++;
            }
        }

        if (std::chrono::duration<double>(now - t_last_print).count() >= 1.0) {
            printf("  [%.0f/%d s]  Depth: %4lu (%.1f fps)  |  RGB: %4lu (%.1f fps)\n",
                   elapsed, duration_sec, (unsigned long)poll_depth, poll_depth / elapsed,
                   (unsigned long)poll_rgb, poll_rgb / elapsed);
            t_last_print = now;
        }
    }

    camera->stopStream(STREAM_TYPE_DEPTH);
    if (found_rgb) camera->stopStream(STREAM_TYPE_RGB);

    double t3_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();

    printf("\n  RESULT (polling):\n");
    printf("    Depth: %lu frames / %.1fs = %.2f fps\n",
           (unsigned long)poll_depth, t3_elapsed, poll_depth / t3_elapsed);
    if (found_rgb)
        printf("    RGB:   %lu frames / %.1fs = %.2f fps\n",
               (unsigned long)poll_rgb, t3_elapsed, poll_rgb / t3_elapsed);

    // =========================================================
    // TEST 4: MJPG stream (compressed — less network bandwidth)
    // =========================================================
    double t4_elapsed = 0;
    uint64_t t4_mjpg = 0;
    double t4_avg_kb = 0;
    double t4_bw_mbps = 0;

    if (found_mjpg) {
        printf("\n  (pause 3s between tests)\n");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        printf("\n========== TEST 4: MJPG callback (compressed stream) ==========\n");
        printf("  Comparing MJPG vs RGB8 to see if compression relieves network load.\n");
        printf("  RGB8 frame size: %dx%dx3 = %.1f MB\n",
               rgb_si.width, rgb_si.height,
               rgb_si.width * rgb_si.height * 3.0 / 1024.0 / 1024.0);
        reset_counters();

        // Start depth + MJPG (instead of RGB8)
        ret = camera->startStream(STREAM_TYPE_DEPTH, depth_si, depth_cb_minimal, nullptr);
        if (ret != SUCCESS) {
            printf("ERROR: depth startStream failed (ret=%d)\n", ret);
        } else {
            ret = camera->startStream(STREAM_TYPE_RGB, mjpg_si, rgb_cb_mjpg, nullptr);
            if (ret != SUCCESS) {
                printf("ERROR: MJPG startStream failed (ret=%d)\n", ret);
                camera->stopStream(STREAM_TYPE_DEPTH);
            } else {
                t_start = std::chrono::steady_clock::now();
                for (int s = 1; s <= duration_sec; ++s) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    double elapsed = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t_start).count();
                    uint64_t mc = mjpg_count.load();
                    uint64_t dc = depth_count.load();
                    uint64_t tb = mjpg_total_bytes.load();
                    int last = mjpg_last_size.load();
                    double avg_kb = mc > 0 ? (tb / (double)mc / 1024.0) : 0;
                    double bw = mc > 0 ? (tb / elapsed / 1024.0 / 1024.0) : 0;
                    printf("  [%2d/%d s]  Depth: %4lu (%.1f fps) | MJPG: %4lu (%.1f fps) "
                           "| last=%d B  avg=%.1f KB  bw=%.1f MB/s\n",
                           s, duration_sec,
                           (unsigned long)dc, dc / elapsed,
                           (unsigned long)mc, mc / elapsed,
                           last, avg_kb, bw);
                }

                camera->stopStream(STREAM_TYPE_DEPTH);
                camera->stopStream(STREAM_TYPE_RGB);

                t4_elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t_start).count();
                t4_mjpg = mjpg_count.load();
                uint64_t total_b = mjpg_total_bytes.load();
                t4_avg_kb = t4_mjpg > 0 ? (total_b / (double)t4_mjpg / 1024.0) : 0;
                t4_bw_mbps = total_b / t4_elapsed / 1024.0 / 1024.0;

                printf("\n  RESULT (MJPG):\n");
                printf("    Depth: %lu frames / %.1fs = %.2f fps\n",
                       (unsigned long)depth_count.load(), t4_elapsed,
                       depth_count.load() / t4_elapsed);
                printf("    MJPG:  %lu frames / %.1fs = %.2f fps\n",
                       (unsigned long)t4_mjpg, t4_elapsed, t4_mjpg / t4_elapsed);
                printf("    MJPG avg frame size: %.1f KB (vs RGB8: %.1f KB)\n",
                       t4_avg_kb,
                       rgb_si.width * rgb_si.height * 3.0 / 1024.0);
                printf("    MJPG bandwidth: %.1f MB/s (vs RGB8 theoretical: %.1f MB/s @ same fps)\n",
                       t4_bw_mbps,
                       t4_mjpg / t4_elapsed * rgb_si.width * rgb_si.height * 3.0 / 1024.0 / 1024.0);
                double compression = (rgb_si.width * rgb_si.height * 3.0 / 1024.0) / t4_avg_kb;
                printf("    Compression ratio: %.1fx\n", compression);
            }
        }
    }

    // =========================================================
    // TEST 5: RGB8-only (NO depth — test if depth caps RGB)
    // =========================================================
    double t5_elapsed = 0;
    uint64_t t5_rgb = 0;

    if (found_rgb) {
        printf("\n  (pause 3s between tests)\n");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        printf("\n========== TEST 5: RGB8-only (NO depth stream) ==========\n");
        printf("  Testing if disabling depth allows higher RGB fps.\n");
        reset_counters();

        ret = camera->startStream(STREAM_TYPE_RGB, rgb_si, rgb_cb_only, nullptr);
        if (ret != SUCCESS) {
            printf("ERROR: RGB8 startStream failed (ret=%d)\n", ret);
        } else {
            t_start = std::chrono::steady_clock::now();
            for (int s = 1; s <= duration_sec; ++s) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t_start).count();
                uint64_t rc = rgb_only_count.load();
                printf("  [%2d/%d s]  RGB8-only: %4lu frames (%.1f fps)\n",
                       s, duration_sec, (unsigned long)rc, rc / elapsed);
            }

            camera->stopStream(STREAM_TYPE_RGB);

            t5_elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            t5_rgb = rgb_only_count.load();

            printf("\n  RESULT (RGB8-only):\n");
            printf("    RGB8: %lu frames / %.1fs = %.2f fps\n",
                   (unsigned long)t5_rgb, t5_elapsed, t5_rgb / t5_elapsed);
            printf("    (compare to Depth+RGB8: %.1f fps)\n",
                   found_rgb ? t1_rgb / t1_elapsed : 0.0);
        }
    }

    // =========================================================
    // TEST 6: MJPG-only (NO depth — compressed + solo)
    // =========================================================
    double t6_elapsed = 0;
    uint64_t t6_mjpg = 0;
    double t6_avg_kb = 0;
    double t6_bw_mbps = 0;

    if (found_mjpg) {
        printf("\n  (pause 3s between tests)\n");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        printf("\n========== TEST 6: MJPG-only (NO depth stream) ==========\n");
        printf("  Testing compressed stream without depth competition.\n");
        reset_counters();

        ret = camera->startStream(STREAM_TYPE_RGB, mjpg_si, mjpg_cb_only, nullptr);
        if (ret != SUCCESS) {
            printf("ERROR: MJPG startStream failed (ret=%d)\n", ret);
        } else {
            t_start = std::chrono::steady_clock::now();
            for (int s = 1; s <= duration_sec; ++s) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t_start).count();
                uint64_t mc = mjpg_only_count.load();
                uint64_t tb = mjpg_only_total_bytes.load();
                int last = mjpg_only_last_size.load();
                double avg_kb = mc > 0 ? (tb / (double)mc / 1024.0) : 0;
                double bw = mc > 0 ? (tb / elapsed / 1024.0 / 1024.0) : 0;
                printf("  [%2d/%d s]  MJPG-only: %4lu (%.1f fps) "
                       "| last=%d B  avg=%.1f KB  bw=%.1f MB/s\n",
                       s, duration_sec,
                       (unsigned long)mc, mc / elapsed,
                       last, avg_kb, bw);
            }

            camera->stopStream(STREAM_TYPE_RGB);

            t6_elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            t6_mjpg = mjpg_only_count.load();
            uint64_t t6_total_b = mjpg_only_total_bytes.load();
            t6_avg_kb = t6_mjpg > 0 ? (t6_total_b / (double)t6_mjpg / 1024.0) : 0;
            t6_bw_mbps = t6_total_b / t6_elapsed / 1024.0 / 1024.0;

            printf("\n  RESULT (MJPG-only):\n");
            printf("    MJPG: %lu frames / %.1fs = %.2f fps\n",
                   (unsigned long)t6_mjpg, t6_elapsed, t6_mjpg / t6_elapsed);
            printf("    avg frame: %.1f KB  bandwidth: %.1f MB/s\n",
                   t6_avg_kb, t6_bw_mbps);
            printf("    (compare to Depth+MJPG: %.1f fps)\n",
                   t4_elapsed > 0 ? t4_mjpg / t4_elapsed : 0.0);
        }
    }

    // ---- Cleanup ----
    camera->disconnect();

    // ---- Summary ----
    printf("\n==================== SUMMARY ====================\n");
    printf("  Expected:          Depth ~%.0f fps   RGB ~%.0f fps\n",
           expected_depth_fps, expected_rgb_fps);
    printf("  --- Depth + RGB combined ---\n");
    printf("  Minimal callback:  Depth %.1f fps   RGB %.1f fps  (RGB8)\n",
           t1_depth / t1_elapsed, found_rgb ? t1_rgb / t1_elapsed : 0.0);
    printf("  Heavy callback:    Depth %.1f fps   RGB %.1f fps  (RGB8)\n",
           t2_depth / t2_elapsed, found_rgb ? t2_rgb / t2_elapsed : 0.0);
    printf("  Polling:           Depth %.1f fps   RGB %.1f fps  (RGB8)\n",
           poll_depth / t3_elapsed, found_rgb ? poll_rgb / t3_elapsed : 0.0);
    if (found_mjpg && t4_elapsed > 0) {
        printf("  MJPG+Depth:        Depth %.1f fps   MJPG %.1f fps\n",
               depth_count.load() / t4_elapsed, t4_mjpg / t4_elapsed);
        printf("                     avg frame: %.1f KB  bw: %.1f MB/s\n",
               t4_avg_kb, t4_bw_mbps);
    }
    printf("  --- Solo streams (no depth) ---\n");
    if (found_rgb && t5_elapsed > 0) {
        printf("  RGB8-only:         %.1f fps\n", t5_rgb / t5_elapsed);
    }
    if (found_mjpg && t6_elapsed > 0) {
        printf("  MJPG-only:         %.1f fps  (avg %.1f KB, bw %.1f MB/s)\n",
               t6_mjpg / t6_elapsed, t6_avg_kb, t6_bw_mbps);
    }
    printf("=================================================\n\n");

    if (t1_depth / t1_elapsed >= expected_depth_fps * 0.8 &&
        (!found_rgb || t1_rgb / t1_elapsed >= expected_rgb_fps * 0.8)) {
        printf("PASS: SDK+hardware deliver expected frame rates.\n");
        printf("      Bottleneck is in ROS node processing.\n");
    } else {
        printf("FAIL: SDK+hardware NOT delivering expected frame rates.\n");
        printf("      Check: USB 3.0 connection, camera firmware, SDK version.\n");
        printf("      Run: lsusb -t   to verify USB speed.\n");
    }

    return 0;
}

/*
 * BUILD:
 *   SDK=/home/ubuntu/ros2_ws/src/revopoint_camera/3DCameraSDK-v3.2.229.20251110
 *   g++ -std=c++17 -O2 \
 *     -I $SDK/inc/3dcamera \
 *     test/sdk_fps_benchmark.cpp \
 *     -L $SDK/lib/3dcamera/linux/x64 \
 *     -l3DCamera -lpthread \
 *     -Wl,-rpath,$SDK/lib/3dcamera/linux/x64 \
 *     -o sdk_fps_benchmark
 *
 * RUN:
 *   ./sdk_fps_benchmark [duration_seconds]
 */
