load("//build_defs:flatbuffers_bundle.bzl", "flatbuffers_bundle")

package(default_visibility = ["//visibility:public"])

exports_files(["*.fbs"])

flatbuffers_bundle(
    name = "rb_v1",
    srcs = glob(["*.fbs"]),
    out_dir = "rb/v1"
)