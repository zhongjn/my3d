#pragma once
#include <stdarg.h>
#include <math.h>
#include <vector>
#include <type_traits>

namespace my3d {

const int X = 0, Y = 1, Z = 2;

#pragma region Vector

template <int _size>
class Vector {
   private:
   public:
       float a[_size];

    Vector() {
        for (register int i = 0; i < _size; i++) {
            a[i] = 0.0f;
        }
    };

    template <typename... Args>
    Vector(Args... args) : a{float(args)...} {
        static_assert(sizeof...(Args) == _size, "Wrong number of arguments!");
    }

    inline float &operator[](int i) { return a[i]; }

    inline float norm() {
        float accum = 0.0f;
        for (register int i = 0; i < _size; i++) {
            accum += a[i] * a[i];
        }
        return sqrtf(accum);
    }

    inline Vector<_size> normalize() { return (*this) / norm(); }

    inline Vector<_size> operator*(float factor) {
        Vector<_size> v2;
        for (register int i = 0; i < _size; i++) {
            v2[i] = a[i] * factor;
        }
        return v2;
    }

    inline Vector<_size> operator/(float factor) {
        return (*this) * (1.0f / factor);
    }

    inline Vector<_size> operator+(Vector<_size> vec) {
        Vector<_size> v2;
        for (register int i = 0; i < _size; i++) {
            v2[i] = a[i] + vec[i];
        }
        return v2;
    }

    inline Vector<_size> operator-() {
        Vector<_size> v2;
        for (register int i = 0; i < _size; i++) {
            v2[i] = -a[i];
        }
        return v2;
    }

    inline Vector<_size> operator-(Vector<_size> vec) {
        return (*this) + (-vec);
    }

    inline float dot(Vector<_size> vec) {
        float accum = 0.0f;
        for (register int i = 0; i < _size; i++) {
            accum += a[i] * vec[i];
        }
        return accum;
    }

    inline float operator*(Vector<_size> vec) { return dot(vec); }
};

#pragma endregion

typedef Vector<2> UV;
typedef Vector<2> ScreenCoordXY;
typedef Vector<3> ScreenCoordXYZ;
typedef Vector<3> Color;
typedef Vector<3> Position;
typedef Vector<3> BilinearCoeffient;

class Matrix3 {
   private:
    float a[3][3] = {0.0f};

   public:
    float *operator[](int i);
    Vector<3> operator*(Vector<3> vec);
    Matrix3 operator*(Matrix3 m);
    Matrix3 operator*(float factor);
    Matrix3 operator/(float factor);
    Matrix3 operator+(Matrix3 m);
    Matrix3 inverse();
    static Matrix3 identity();
    static Matrix3 from_rotation(Vector<3> rotation);
    static Matrix3 from_scale(float scale);
};

class Transformation {
   public:
    Transformation();
    Transformation(Matrix3 linear, Vector<3> translation);
    Transformation inverse();
    Transformation operator*(Transformation t);
    Vector<3> apply(Vector<3> v, bool affine);
    Transformation scale(float scale);
    Transformation rotate(int axis, float theta);
    Transformation translate(float x, float y, float z);
    Transformation translate(Vector<3> vec);
    Matrix3 linear;
    Vector<3> translation;
};

struct Size {
    float width, height;
};

//顶点
struct Vertex {
    friend class Context;

   public:
    Position position;
    Color color;

   private:
    ScreenCoordXYZ coord;
};

struct Triangle {
    friend class Context;

   public:
    int index[3] = {0};
    Triangle();
    Triangle(int i0, int i1, int i2);

   private:
    bool backface_cull = false;
    bool culled = false;
    bool normal_computed = false;
    float roughness;
    bool accept_light;
    Vector<3> normal;
};

//像素
struct Pixel {
    friend class Context;

   public:
    float depth = 0.0;  //深度
                        // Color color = { 0, 0, 0 };
   private:
    Triangle *triangle = nullptr;
    BilinearCoeffient bilinear_coefficient;
};

// 3D物体最基本的单位
struct Object {
   public:
    Transformation transformation;
};

struct Light {
   public:
    Color color = {255, 255, 255};
    float intensity = 1.0f;
};

struct AmbientLight : public Light {};

struct PointLight : public Light {
    Position position;
};

struct Mesh : public Object {
   public:
    std::vector<Vertex> vertexes;
    std::vector<Triangle> triangles;
    bool backface_cull = false;
    float roughness = 1.0f;
    bool accept_light = true;
    Mesh();
    Mesh(std::vector<Vertex> vertexes, std::vector<Triangle> triangles);
};

struct Camera : public Object {
   public:
    Camera();
    Camera(Transformation trans, Size lens);
    Size lens_size;
    float depth_minimum = 1.0f;
};

class Context {
   public:
    short height, width;
    Context(short width, short height);  //根据画面高度宽度初始化pixels
    inline Color get_scene_output(int x, int y) {
        return output[y * width + x];
    }
    void scene_begin();  //开始画一帧，还原pixels
    void scene_begin(Camera cam);
    void scene_end();  //结束画一帧
    void draw_mesh(Mesh &mesh);
    void set_ambient_light(AmbientLight &light);
    void set_point_light(PointLight &light);
    void set_point_light(std::vector<PointLight> &lights);
    void set_world_transformation(Transformation world);
    Camera camera;

   private:
    // 私有变量
    Pixel *buffer_pixels;
    Color *output;
    std::vector<Vertex> vertexes;
    std::vector<Triangle> triangles;
    AmbientLight ambient_light;
    std::vector<PointLight> point_lights;
    Transformation transformation_combined;
    // 工具函数
    Pixel &get_pixel(int i, int j);
    ScreenCoordXYZ project_to_screen(Position pos);
    // 渲染管线
    void vertex_shade();
    void cull();
    void clip_z();
    void rasterize();
    void pixel_shade();
    void compute_triangle_normal(Triangle *triangle);
};

}  // namespace my3d