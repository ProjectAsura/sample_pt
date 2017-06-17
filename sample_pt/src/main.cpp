//-------------------------------------------------------------------------------------------------
// File : main.cpp
// Desc : Main Entry Point.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Inlcudes
//-------------------------------------------------------------------------------------------------
#include <r3d_math.h>
#include <r3d_camera.h>
#include <r3d_shape.h>
#include <vector>
#include <stb/stb_image_write.h>


namespace {

//-------------------------------------------------------------------------------------------------
// Global Varaibles.
//-------------------------------------------------------------------------------------------------
const int     g_max_depth = 5;
const Vector3 g_light_pos   (50.0,  75.0,   81.6);
const Vector3 g_light_color (256.0, 256.0, 256.0);
const Vector3 g_back_ground (0.0,   0.0,    0.0);
const Vector3 g_shadow_color(0.0,   0.0,    0.0);
const Sphere  g_spheres[] = {
    Sphere(1e5,     Vector3( 1e5 + 1.0,    40.8,          81.6), Vector3(0.25,  0.75,  0.25), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(-1e5 + 99.0,   40.8,          81.6), Vector3(0.25,  0.25,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,          40.8,           1e5), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,          40.8,  -1e5 + 170.0), Vector3(),                   ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,           1e5,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,   -1e5 + 81.6,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(16.5,    Vector3(27.0,          16.5,          47.0), Vector3(0.75,  0.25,  0.25), ReflectionType::Specular),
    Sphere(16.5,    Vector3(73.0,          16.5,          78.0), Vector3(0.99,  0.99,  0.99), ReflectionType::Refraction)
};


//-------------------------------------------------------------------------------------------------
//      シーンとの交差判定を行います.
//-------------------------------------------------------------------------------------------------
inline bool intersect_scene(const Ray& ray, double* t, int* id)
{
    auto n = static_cast<int>(sizeof(g_spheres) / sizeof(g_spheres[0]));

    *t  = D_MAX;
    *id = -1;

    for (auto i = 0; i < n; ++i)
    {
        auto d = g_spheres[i].intersect(ray);
        if (d > D_HIT_MIN && d < *t)
        {
            *t  = d;
            *id = i;
        }
    }

    return (*t < D_HIT_MAX);
}

//-------------------------------------------------------------------------------------------------
//      放射輝度を求めます.
//-------------------------------------------------------------------------------------------------
Vector3 radiance(const Ray& ray, int depth)
{
    double t;
    int   id;

    // シーンとの交差判定.
    if (!intersect_scene(ray, &t, &id))
    { return g_back_ground; }

    // 交差物体.
    const auto& obj = g_spheres[id];

    // 交差位置.
    const auto hit_pos = ray.pos + ray.dir * t;

    // 法線ベクトル.
    const auto normal  = normalize(hit_pos - obj.pos);

    // 物体からのレイの入出を考慮した法線ベクトル.
    const auto orienting_normal = (dot(normal, ray.dir) < 0.0) ? normal : -normal;

    // 打ち切り深度に達したら終わり.
    if(depth > g_max_depth)
    { return g_back_ground; }

    switch (obj.type)
    {
    case ReflectionType::Diffuse:
        {
            double t_;
            int    id_;

            // ライトベクトル.
            auto light_dir  = g_light_pos - hit_pos;

            // ライトまでの距離.
            auto light_dist = length(light_dir);

            // ライトベクトルを正規化.
            light_dir /= light_dist;

            // ライトとの間に遮蔽物がないことを確認.
            intersect_scene(Ray(hit_pos, light_dir), &t_, &id_);

            // 遮蔽物がない場合.
            if (t_ >= light_dist)
            {
                auto diffuse = obj.color * max(dot(orienting_normal, light_dir), 0.0) / (light_dist * light_dist);
                return g_light_color * diffuse;
            }
            else
            {
                // 遮蔽物がある.
                return g_shadow_color;
            }
        }
        break;

    case ReflectionType::PerfectSpecular:
        {
            // 反射させる.
            return obj.color * radiance(Ray(hit_pos, reflect(ray.dir, normal)), depth + 1);
        }
        break;

    case ReflectionType::Refraction:
        {
            // 反射レイ
            auto reflect_ray = Ray(hit_pos, reflect(ray.dir, normal));

            // 内部侵入するか?
            auto into = dot(normal, orienting_normal) > 0.0;

            // 空気の屈折率
            const auto nc = 1.0;

            // 物体の屈折率
            const auto nt = 1.5;

            // Snellの法則.
            const auto nnt = (into) ? (nc / nt) : (nt / nc);
            const auto vn  = dot(ray.dir, orienting_normal);
            const auto cos2t = 1.0 - nnt * nnt * (1.0 - vn * vn);

            // 全反射かどうかチェック.
            if (cos2t < 0.0)
            { return obj.color * radiance(reflect_ray, depth + 1); }

            // 屈折ベクトル.
            auto refract = normalize(ray.dir * nnt - normal * ((into) ? 1.0 : -1.0) * (vn * nnt + sqrt(cos2t)) );

            // Schlickによる Fresnel の反射係数の近似.
            const auto a  = nt - nc;
            const auto b  = nt + nc;
            const auto R0 = (a * a) / (b * b);

            const auto c  = 1.0 - ((into) ? -vn : dot(refract, normal));
            const auto Re = R0 + (1.0 - R0) * pow(c, 5.0);

            const auto nnt2 = pow((into) ? (nc / nt) : (nt /nc), 2.0);
            const auto Tr = (1.0 - Re) * nnt2;
            const auto p  = 0.25 + 0.5 * Re;

            // 屈性レイ
            Ray refract_ray(hit_pos, refract);

            const auto reflect_result = radiance(reflect_ray, depth + 1) * Re;
            const auto refract_result = radiance(refract_ray, depth + 1) * Tr;

            return obj.color * (reflect_result + refract_result);
        }
        break;

    case ReflectionType::Specular:
        {
            double t_;
            int    id_;

            // ライトベクトル.
            auto light_dir  = g_light_pos - hit_pos;

            // ライトまでの距離.
            auto light_dist = length(light_dir);

            // ライトベクトルを正規化.
            light_dir /= light_dist;

            // ライトまでの間に遮蔽物がないかどうかチェック.
            intersect_scene(Ray(hit_pos, light_dir), &t_, &id_);
            if (t_ >= light_dist)
            {
                auto shininess  = 500.0;
                auto diffuse    = (obj.color / (light_dist * light_dist)) * max(dot(orienting_normal, light_dir), 0.0);
                auto specular   = (obj.color) * pow(max(dot(light_dir, reflect(ray.dir, orienting_normal)), 0.0), shininess);
                return g_light_color * (diffuse + specular);
            }
            else
            {
                // 遮蔽物がある.
                return g_shadow_color;
            }
        }
        break;
    }

    // どれにもヒットしなかった.
    return g_back_ground;
}

//-------------------------------------------------------------------------------------------------
//      BMPファイルに保存します.
//-------------------------------------------------------------------------------------------------
void save_to_bmp(const char* filename, int width, int height, const double* pixels)
{
    std::vector<uint8_t> images;
    images.resize(width * height * 3);

    const double inv_gamma = 1.0 / 2.2;

    for(auto i=0; i<width * height * 3; i+=3)
    {
        auto r = pow(pixels[i + 0], inv_gamma);
        auto g = pow(pixels[i + 1], inv_gamma);
        auto b = pow(pixels[i + 2], inv_gamma);

        r = saturate(r);
        g = saturate(g);
        b = saturate(b);

        images[i + 0] = static_cast<uint8_t>( r * 255.0 + 0.5 );
        images[i + 1] = static_cast<uint8_t>( g * 255.0 + 0.5 );
        images[i + 2] = static_cast<uint8_t>( b * 255.0 + 0.5 );
    }

    stbi_write_bmp(filename, width, height, 3, images.data());
}

} // namespace


//-------------------------------------------------------------------------------------------------
//      メインエントリーポイントです.
//-------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // レンダーターゲットのサイズ.
    int width  = 640;
    int height = 480;

    // カメラ用意.
    Camera camera(
        Vector3(50.0, 52.0, 295.6),
        normalize(Vector3(0.0, -0.042612, -1.0)),
        Vector3(0.0, 1.0, 0.0),
        0.5135,
        double(width) / double(height),
        130.0
    );

    // レンダーターゲット生成.
    std::vector<Vector3> image;
    image.resize(width * height);

    // レンダーターゲットをクリア.
    for (size_t i = 0; i < image.size(); ++i)
    { image[i] = g_back_ground; }

    for (auto y = 0; y < height; ++y)
    {
        for (auto x = 0; x < width; ++x)
        {
            auto idx = y * width + x;
            auto fx = double(x) / double(width)  - 0.5;
            auto fy = double(y) / double(height) - 0.5;

            // Let's レイトレ！
            image[idx] += radiance(camera.emit(fx, fy), 0);
        }
    }

    // レンダーターゲットの内容をファイルに保存.
    save_to_bmp("image.bmp", width, height, &image.data()->x);

    // レンダーターゲットクリア.
    image.clear();

    return 0;
}