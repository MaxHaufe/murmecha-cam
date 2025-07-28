#ifndef CAMERA
#define CAMERA

esp_err_t init_camera(void);

class MurmechaCam;

struct ImageData {
    uint8_t *buf;
    size_t len;
    size_t width;
    size_t height;

    ImageData() : buf(nullptr), len(0), width(0), height(0) {
    }

    ImageData(uint8_t *b, const size_t l, const size_t w, const size_t h)
        : buf(b), len(l), width(w), height(h) {
    }

    ~ImageData();
};

class MurmechaCam {

public:
    MurmechaCam();

    ~MurmechaCam();

    static esp_err_t init_camera();

    static ImageData get_rgb_image();
};

#endif /* CAMERA */
