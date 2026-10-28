// fluent_lib/hw_jpeg.hpp の確かめ (ROS 無し): 専用回路 (NVJPG) と OpenCV で同じ JPEG を開く・作り、
// 時間 (壁時計・CPU) と絵の差を出す。回転 (VIC) も比べる。
//   hw_jpeg_check [--rotation 90] [--quality 85] [--iters N] file.jpg...
#include <fluent_lib/hw_jpeg.hpp>
#include <sys/resource.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

static double cpu_ms() { rusage r{}; getrusage(RUSAGE_SELF, &r); return (r.ru_utime.tv_sec + r.ru_stime.tv_sec)*1000.0 + (r.ru_utime.tv_usec + r.ru_stime.tv_usec)/1000.0; }
static double mean_diff(const cv::Mat& a, const cv::Mat& b) {
  if (a.size() != b.size() || a.type() != b.type()) return -1;
  cv::Mat d; cv::absdiff(a, b, d); const auto m = cv::mean(d); return (m[0]+m[1]+m[2])/3;
}
int main(int argc, char** argv) {
  int rotation = 0, quality = 85, iters = 3;
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--rotation" && i+1 < argc) rotation = std::stoi(argv[++i]);
    else if (a == "--quality" && i+1 < argc) quality = std::stoi(argv[++i]);
    else if (a == "--iters" && i+1 < argc) iters = std::stoi(argv[++i]);
    else files.push_back(a);
  }
  if (files.empty()) { std::fprintf(stderr, "usage: hw_jpeg_check [--rotation 90] [--quality 85] [--iters N] file.jpg...\n"); return 2; }
  std::vector<std::vector<uint8_t>> jpegs;
  for (auto& f : files) { std::ifstream in(f, std::ios::binary); jpegs.emplace_back((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()); }
  std::string why;
  std::printf("hardware=%d %s\n", fluent::hwjpeg::Decoder::hardware_available(&why), why.c_str());
  cv::Mat ref; { fluent::hwjpeg::Decoder d(false); d.decode(jpegs[0].data(), jpegs[0].size(), ref, rotation); }
  for (bool hw : {true, false}) {
    fluent::hwjpeg::Decoder dec(hw);
    cv::Mat out; std::string err;
    if (!dec.decode(jpegs[0].data(), jpegs[0].size(), out, rotation, 0, 0, &err)) { std::printf("decode %s failed: %s\n", dec.backend(), err.c_str()); continue; }
    const double diff = mean_diff(ref, out);
    const double c0 = cpu_ms(); const auto t0 = std::chrono::steady_clock::now(); int n = 0;
    for (int it = 0; it < iters; ++it) for (auto& j : jpegs) if (dec.decode(j.data(), j.size(), out, rotation)) ++n;
    const double wall = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-t0).count()/n, cpu = (cpu_ms()-c0)/n;
    std::printf("decode backend=%s last=%s rotation=%d out=%dx%d wall=%.2f ms cpu=%.2f ms diff_vs_opencv=%.3f err=%s\n",
                dec.backend(), dec.last_backend(), rotation, out.cols, out.rows, wall, cpu, diff, dec.hardware_error().c_str());
  }
  for (bool hw : {true, false}) {
    fluent::hwjpeg::Encoder enc(hw);
    std::vector<uint8_t> out; std::string err;
    if (!enc.encode(ref, quality, out, &err)) { std::printf("encode %s failed: %s\n", enc.backend(), err.c_str()); continue; }
    cv::Mat back; { fluent::hwjpeg::Decoder d(false); d.decode(out.data(), out.size(), back); }
    const double diff = mean_diff(ref, back);
    const double c0 = cpu_ms(); const auto t0 = std::chrono::steady_clock::now(); const int n = iters*int(jpegs.size());
    for (int k = 0; k < n; ++k) enc.encode(ref, quality, out);
    const double wall = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-t0).count()/n, cpu = (cpu_ms()-c0)/n;
    std::printf("encode backend=%s last=%s in=%dx%d bytes=%zu wall=%.2f ms cpu=%.2f ms diff_vs_source=%.3f err=%s\n",
                enc.backend(), enc.last_backend(), ref.cols, ref.rows, out.size(), wall, cpu, diff, enc.hardware_error().c_str());
  }
  return 0;
}
