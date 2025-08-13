#ifndef CAMERA
#define CAMERA
#include <memory>

esp_err_t init_camera(void);

class MurmechaCam;

class ImageData {
// private:


public:
    std::shared_ptr<uint8_t[]> buf;  // Shared ownership
    size_t len;
    int width, height;
    ImageData(uint8_t* buffer, size_t length, int w, int h)
        : buf(buffer, free), len(length), width(w), height(h) {}  // Custom deleter

    // Copy constructor works automatically - reference counting handles memory
    // Destructor works automatically - memory freed when last shared_ptr is destroyed
};

class MurmechaCam {

public:
    MurmechaCam();

    ~MurmechaCam();

    static esp_err_t init_camera();

    static ImageData get_rgb_image();
};

#endif /* CAMERA */
