import os

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import copy, get


class GoPiGo3Conan(ConanFile):
    name = "gopigo3"
    version = "1.0.3"
    package_type = "shared-library"
    license = "MIT"
    homepage = "https://github.com/DexterInd/GoPiGo3"
    description = "C++ driver for the Dexter Industries GoPiGo3 Raspberry Pi robot"
    topics = ("robotics", "raspberry-pi", "gopigo3", "spi", "differential-drive")
    settings = "os", "arch", "compiler", "build_type"

    def layout(self):
        cmake_layout(self, src_folder="src")
        # Upstream keeps the public header next to its CMake project, not in include/.
        self.cpp.source.includedirs = ["Software/C"]

    def validate(self):
        if self.settings.os != "Linux":
            raise ConanInvalidConfiguration(
                "gopigo3 drives the robot through /dev/spidev, only available on Linux"
            )

    def source(self):
        get(self, **self.conan_data["sources"][self.version], strip_root=True)

    def generate(self):
        tc = CMakeToolchain(self)
        # Upstream declares cmake_minimum_required(VERSION 3.0), rejected by CMake >= 4.
        tc.cache_variables["CMAKE_POLICY_VERSION_MINIMUM"] = "3.5"
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure(build_script_folder=os.path.join("Software", "C"))
        # Only the library: the upstream default target also builds the SPI examples.
        cmake.build(target="gopigo3")

    def package(self):
        copy(self, "LICENSE.md", self.source_folder,
             os.path.join(self.package_folder, "licenses"))
        copy(self, "GoPiGo3.h", os.path.join(self.source_folder, "Software", "C"),
             os.path.join(self.package_folder, "include"))
        copy(self, "*.so*", self.build_folder,
             os.path.join(self.package_folder, "lib"), keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["gopigo3"]
        # Mirror the config and target name that upstream's own install() exports, so
        # consumers written against a system GoPiGo3 build need no CMake changes.
        self.cpp_info.set_property("cmake_file_name", "gopigo3_cpp")
        self.cpp_info.set_property("cmake_target_name", "gopigo3")
