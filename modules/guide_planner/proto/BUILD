load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "trajectory_cc_proto",
    deps = [
        ":trajectory_proto",
    ],
)

proto_library(
    name = "trajectory_proto",
    srcs = ["Trajectory.proto"],
)

cpplint()
