load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "guide_filter.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":guide_filterlib"],
)

cc_library(
    name = "guide_filterlib",
    srcs = ["guide_filter.cc"],
    hdrs = ["guide_filter.h"],
    deps = [
        "//cyber",
    	"//modules/guide_can/proto:chassis_detail_cc_proto",
        ":filterlib"
    ],
)

cc_library(
    name = "filterlib",
    srcs = ["filter.h"],
)


cpplint()
