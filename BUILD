load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

#cc_binary(
#    name = "main",
#    srcs = ["main.cpp"],
#    deps = ["@bazel_tools//tools/cpp/runfiles", ":atcoder"],
#    data = ["t_mons.tbl"],
#    copts = ["-Xclang", "-Wno-c++17-extensions", "-O2", "-Wno-unqualified-std-cast-call"],
#    linkopts = ["-Xclang", "-Wl,-export_dynamic", "-g"],
#    features = [ "fully_static_link" ],
#)

#cc_library(
#        name = "atcoder",
#        hdrs =["bits/stdc++.h"] + glob(["atcoder/*"]),
#)

cc_library(
    name = "vector",
    hdrs = ["vector.h"],
)

cc_library(
    name = "math_util",
    hdrs = ["math_util.h"],
    srcs = ["math_util.cc"],
    deps = [":vector"],
)

cc_test(
    name = "math_util_test",
    srcs = ["math_util_test.cc"],
    deps = [
        ":math_util",
        "@googletest//:gtest_main"
    ],
)
