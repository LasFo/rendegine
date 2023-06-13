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
    name = "vector3d",
    hdrs = ["vector3d.h"],
)

cc_library(
    name = "math_util",
    hdrs = ["math_util.h"],
    srcs = ["math_util.cc"],
    deps = [":vector3d"],
)

cc_test(
    name = "math_util_test",
    srcs = ["math_util_test.cc"],
    deps = [
        ":math_util",
        "@googletest//:gtest_main"
    ],
)

cc_library(
    name = "euler_angles",
    hdrs = ["euler_angles.h"],
    srcs = ["euler_angles.cc"],
    deps = [
        ":math_util",
        ":quaternion",
        ":matrix_4x3",
        ":rotation_matrix",
    ],
)

cc_test(
    name = "euler_angles_test",
    srcs = ["euler_angles_test.cc"],
    deps = [
        ":euler_angles",
        ":math_util",
        ":quaternion",
        ":matrix_4x3",
        ":rotation_matrix",
        "@googletest//:gtest_main"
    ],
)

cc_library(
    name = "quaternion",
    hdrs = ["quaternion.h"],
)

cc_library(
    name = "matrix_4x3",
    hdrs = ["matrix_4x3.h"],
)

cc_library(
    name = "rotation_matrix",
    hdrs = ["rotation_matrix.h"],
)
