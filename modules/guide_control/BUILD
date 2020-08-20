load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "guide_control.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":guide_controllib"],
)

cc_library(
    name = "guide_controllib",
    srcs = ["guide_control.cc"],
    hdrs = ["guide_control.h"],
    deps = [
        "//cyber",
        
	"//modules/guide_can/vehicle:guide_controller",
	"//modules/guide_can/proto:chassis_detail_cc_proto",
	"//modules/guide_can/proto:control_command_cc_proto",
    "//modules/guide_planner/proto:trajectory_cc_proto",
    "//modules/guide_control/controller:PIDcontrollerlib"
    ],
)

cpplint()
