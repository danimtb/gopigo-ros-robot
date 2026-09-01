#include <iostream>
#include <stdexcept>

#include <GoPiGo3.h>

int main()
{
    try {
        GoPiGo3 gpg;
        // detect(false) returns an error code instead of throwing when no board answers,
        // so a machine without a GoPiGo3 attached still exercises the SPI code path.
        const int error = gpg.detect(false);
        std::cout << "GoPiGo3 detect: " << error
                  << ", wheel base " << gpg.WHEEL_BASE_WIDTH << " mm"
                  << ", wheel diameter " << gpg.WHEEL_DIAMETER << " mm\n";
    } catch (const std::runtime_error & e) {
        // Build machines have no /dev/spidev0.1, so the constructor throws. Reaching this
        // point already proves the library links and its symbols resolve at runtime.
        std::cout << "GoPiGo3 not reachable: " << e.what() << "\n";
    }
    return 0;
}
