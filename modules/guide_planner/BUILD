load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "guide_planner.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":guide_plannerlib"],
)

cc_library(
    name = "guide_plannerlib",
    srcs = ["guide_planner.cc"],
    hdrs = ["guide_planner.h"],
    deps = [
	":Plannerlib",
        "//cyber",
	"//modules/guide_can/proto:chassis_detail_cc_proto",
        "//modules/guide_planner/proto:trajectory_cc_proto"
    ],
)

cc_library(
    name = "Plannerlib",
    srcs = ["Planner.cc"],
    hdrs = ["Planner.h"],
    deps = [],
)

cpplint()
