from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class GoPiGo3RosConan(ConanFile):
    name = "gopigo3_ros"
    version = "1.0.0"
    package_type = "application"
    license = "MIT"
    description = "ROS 2 node that turns geometry_msgs/Twist into GoPiGo3 wheel speeds"
    topics = ("robotics", "ros2", "gopigo3", "differential-drive")
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "CMakeLists.txt", "src/*", "relocate_env.sh"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("ros-kilted/2026.06.17")
        self.requires("gopigo3/1.0.3")

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
