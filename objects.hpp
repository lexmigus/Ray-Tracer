#include <string>
#include "raytrace.hpp"

#define ZERO 0.0

//Lights class
struct Lights {
        std::string name;
        Vec position;
        Vec colour;

        Lights() = default;

        Lights(const std::string &name, const Vec &position, const Vec &colour)
                : name(name), position(position), colour(colour) {}
};

// Sphere class
struct Sphere {
        std::string name;
        Vec position;
        Vec scale;
        Vec colour;
        double ka = ZERO; // Ambient reflection coefficient
        double kd = ZERO; // Diffuse reflection coefficient
        double ks = ZERO; // Specular reflection coefficient
        double kr = ZERO; // Reflection coefficient
        double n = ZERO;  // Specular exponent

        Sphere(const std::string &name, const Vec &position, const Vec &scale,
                   const Vec &colour, double ka, double kd, double ks, double kr, double n)
                : name(name), position(position), scale(scale), colour(colour), ka(ka), kd(kd), ks(ks), kr(kr), n(n) {}

        Sphere() = default;
};

// Scene Class
struct Scene {
        double near = ZERO, left = ZERO, right = ZERO, bottom = ZERO, top = ZERO, width = ZERO, height = ZERO;
        std::vector<Sphere> spheres;
        std::vector<Lights> lights;
        Vec bgColour;
        Vec ambientColour;
        std::string outputName;

        Scene() = default;

        Scene(double near, double left, double right, double bottom, double top,
            double width, double height,
            std::vector<Sphere> spheres,
            std::vector<Lights> lights,
            const Vec& bgColour,
            const Vec& ambientColour,
            const std::string& outputName)
          : near(near), left(left), right(right), bottom(bottom), top(top),
            width(width), height(height),
            spheres(std::move(spheres)), lights(std::move(lights)),
            bgColour(bgColour),
            ambientColour(ambientColour),
            outputName(outputName) {}
};