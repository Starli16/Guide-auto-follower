load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "guide_can.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":guide_canlib"],
)

cc_library(
    name = "guide_canlib",
    srcs = ["guide_can.cc"],
    hdrs = ["guide_can.h"],
    deps = [
        "//cyber",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
        "//modules/drivers/canbus/proto:can_card_parameter_cc_proto",
	"//modules/drivers/canbus/can_comm:can_receiver",
	"//modules/drivers/canbus/can_comm:can_sender",
	"//modules/guide_can/vehicle:guide_message_manager",
	"//modules/guide_can/vehicle:guide_controller",
    ],
)

cpplint()
