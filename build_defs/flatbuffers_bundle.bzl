load("@flatbuffers//:build_defs.bzl", "flatbuffer_library_public")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_python//python:py_library.bzl", "py_library")

def _basename_no_ext(path):
    """Returns the base name of a file without its extension."""
    return path.rsplit("/")[-1].removesuffix(".fbs")

def flatbuffers_bundle(name, srcs, out_dir = "", visibility = None):
    """
    Generates Flatbuffers for multiple .fbs and exposes:
      :<name>_cc (cc_library)
      :<name>_py (py_library)
    """
    if out_dir and out_dir.endswith("/"):
        out_dir = out_dir[:-1]

    py_outs = []
    cc_outs = []

    for s in srcs:
        base = _basename_no_ext(s)
        py_out = "{}/{}_fb.py".format(out_dir, base) if out_dir else "{}_fb.py".format(base)
        cc_out = "{}/{}_generated.h".format(out_dir, base) if out_dir else "{}_generated.h".format(base)

        py_outs.append(py_out)
        cc_outs.append(cc_out)

        flatbuffer_library_public(
            name = "{}_{}_py".format(name, base),
            srcs = [s],
            outs = [py_out],
            flatc_args = [
                "--gen-onefile",
                "--filename-suffix",
                "_fb",
                "--filename-ext",
                "py",
            ],
            language_flag = "--python",
            visibility = visibility,
        )

        flatbuffer_library_public(
            name = "{}_{}_cc".format(name, base),
            srcs = [s],
            outs = [cc_out],
            flatc_args = [
                "--filename-suffix",
                "_generated",
                "--filename-ext",
                "h",
            ],
            language_flag = "--cpp",
            visibility = visibility,
        )

    cc_library(
        name = "{}_cc".format(name),
        hdrs = cc_outs,
        includes = ["."],
        visibility = visibility,
    )

    py_library(
        name = "{}_py".format(name),
        srcs = py_outs,
        imports = ["."],
        visibility = visibility,
    )
