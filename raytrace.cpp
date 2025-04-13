using namespace std;

#include <fstream>
#include <sstream>
#include <limits>
#include "objects.hpp"


// Look for intersections
bool intersection(const Ray &ray, const Sphere &sphere, double &t, bool &inside)
{
    Vec scaledOrigin = (ray.origin - sphere.position) / sphere.scale;
    Vec scaledDirection = ray.direction / sphere.scale;

    double a = scaledDirection.dot(scaledDirection);
    double b = 2.0 * scaledOrigin.dot(scaledDirection);
    double col = scaledOrigin.dot(scaledOrigin) - 1.0;
    double discriminant = pow(b, 2) - 4 * a * col;

    double rayLength = ZERO;
    bool is_origin_ray = ray.origin == Vec(0, 0, 0);
    if (is_origin_ray)
    {
        rayLength = ray.direction.length();
    }

    if (discriminant < 0)
    {
        return false; // No intersection
    }

    double sqrtDiscriminant = sqrt(discriminant);
    double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
    double t2 = (-b + sqrtDiscriminant) / (2.0 * a);

    if (t1 > 0 && t1 > rayLength || t2 > 0 && t2 > rayLength)
    if ((t1 > rayLength && t1 > 0) || (t2 > rayLength && t2 > 0))
        {
            t = min(t1, t2);
            // Check if inside Sphere, -1 is near plane
            if ((t1 - 1) * (t2 - 1) <= 0 && is_origin_ray)
            {
                t = max(t1, t2);
                inside = true;
            }

            return true;
        }

    return false; // Intersection points are behind the ray
}

// Find the closest intersection among spheres
bool nearest_intersection(const Ray &ray, const vector<Sphere> &spheres, Sphere &closestSphere, double &t, bool &inside){
    t = numeric_limits<double>::infinity();
    bool foundIntersection = false;

    for (const auto &sphere : spheres){
        double currentT;
        bool currentInside;

        if (intersection(ray, sphere, currentT, currentInside) && currentT < t){
            t = currentT;
            closestSphere = sphere;
            inside = currentInside;
            foundIntersection = true;
        }
    }

    return foundIntersection;
}

// Compute ambient light
Vec computeAmbient(const Scene &scene, const Sphere &sphere) {
    return scene.ambientColour * sphere.ka * sphere.colour;
}

// Check if the point is in shadow
bool isInShadow(const Vec &point, const Vec &lightDir, const Scene &scene, const Lights &light, bool inside) {
    Ray shadowRay(point + lightDir * 0.00001, lightDir, 0);
    Sphere obstruction;
    double t;
    bool shadowInside = false;
    bool blocked = nearest_intersection(shadowRay, scene.spheres, obstruction, t, shadowInside);
    return blocked && !(inside && light.position == obstruction.position);
}

// Compute diffuse light
Vec computeDiffuse(const Lights &light, const Sphere &sphere, const Vec &normal, const Vec &lightDir) {
    double NdotL = normal.dot(lightDir);
    return (NdotL > 0) ? light.colour * sphere.kd * NdotL * sphere.colour : Vec(0, 0, 0);
}

// Compute specular light
Vec computeSpecular(const Lights &light, const Sphere &sphere, const Vec &normal, const Vec &lightDir, const Vec &viewDir) {
    Vec reflectDir = (2.0 * normal.dot(lightDir) * normal - lightDir).normalize();
    double RdotV = std::max(ZERO, reflectDir.dot(viewDir));
    return light.colour * sphere.ks * pow(RdotV, sphere.n);
}

// Calculate the lighting at the intersection point
Vec calculateLighting(const Ray &ray, const Scene &scene, const Sphere &sphere, const Vec &hitPoint, Vec &normal, bool &inside) {
    Vec ambient = computeAmbient(scene, sphere);
    Vec diffuseSum(0, 0, 0);
    Vec specularSum(0, 0, 0);
    Vec viewDir = (ray.origin - hitPoint).normalize();

    for (const auto &light : scene.lights) {
        Vec lightDir = (light.position - hitPoint).normalize();

        if (!isInShadow(hitPoint, lightDir, scene, light, inside)) {
            diffuseSum += computeDiffuse(light, sphere, normal, lightDir);
            specularSum += computeSpecular(light, sphere, normal, lightDir, viewDir);
        }
    }

    return ambient + diffuseSum + specularSum;
}


Vec computeRay(const Ray &initialRay, const Scene &scene) {
    const unsigned int MAX_DEPTH = 3;

    Ray ray = initialRay;
    Vec accumulatedColor(0, 0, 0);
    Vec reflectivity(1, 1, 1); // Tracks reflection contribution per depth

    for (unsigned int depth = 0; depth <= MAX_DEPTH; ++depth) {
        Sphere closestSphere;
        double t = ZERO;
        bool inside = false;

        if (!nearest_intersection(ray, scene.spheres, closestSphere, t, inside)) {
            Vec bg = (ray.origin == Vec(0, 0, 0)) ? scene.bgColour : Vec(0, 0, 0);
            accumulatedColor += reflectivity * bg;
            break;
        }

        // Calculate local intersection point
        Vec hitPoint = ray.origin + ray.direction * t;

        // Compute normal at the intersection point
        Vec squaredScale = Vec(
            closestSphere.scale.x * closestSphere.scale.x,
            closestSphere.scale.y * closestSphere.scale.y,
            closestSphere.scale.z * closestSphere.scale.z
        );
        Vec normal = ((hitPoint - closestSphere.position) / squaredScale).normalize();
        if (inside) {
            normal = normal.opposite();
        }

        // Calculate local shading
        Vec localColor = calculateLighting(ray, scene, closestSphere, hitPoint, normal, inside);
        accumulatedColor += reflectivity * localColor;

        // Compute reflection direction and update ray
        Vec reflectionDir = (-2.0 * normal.dot(ray.direction) * normal + ray.direction).normalize();
        ray = Ray(hitPoint + reflectionDir * 0.00001, reflectionDir, ray.depth + 1);

        // Scale reflection contribution
        reflectivity *= closestSphere.kr;
    }

    return accumulatedColor.clamp();
}


// Split file info into tokens
vector<string> tokenize(const string &str, const string &delimiters)
{
    vector<string> tokens;
    size_t start = 0, end;

    while ((end = str.find_first_of(delimiters, start)) != string::npos)
    {
        string token = str.substr(start, end - start);
        if (!token.empty()) tokens.push_back(token);
        start = end + 1;
    }

    string lastToken = str.substr(start);
    if (!lastToken.empty()) tokens.push_back(lastToken);

    return tokens;
}

Scene readFile(const string &filename)
{
    ifstream inputFile(filename);
    Scene scene;
    string line;
    while (getline(inputFile, line))
    {
        auto tokens = tokenize(line, " \t");
        if (tokens.empty()) continue;

        if (tokens[0] == "NEAR") scene.near = stod(tokens[1]);
        else if (tokens[0] == "LEFT") scene.left = stod(tokens[1]);
        else if (tokens[0] == "RIGHT") scene.right = stod(tokens[1]);
        else if (tokens[0] == "BOTTOM") scene.bottom = stod(tokens[1]);
        else if (tokens[0] == "TOP") scene.top = stod(tokens[1]);
        else if (tokens[0] == "RES") { scene.width = stod(tokens[1]); scene.height = stod(tokens[2]); }
        else if (tokens[0] == "SPHERE")
            scene.spheres.emplace_back(tokens[1], Vec(stod(tokens[2]), stod(tokens[3]), stod(tokens[4])),
                                        Vec(stod(tokens[5]), stod(tokens[6]), stod(tokens[7])),
                                        Vec(stod(tokens[8]), stod(tokens[9]), stod(tokens[10])),
                                        stod(tokens[11]), stod(tokens[12]), stod(tokens[13]),
                                        stod(tokens[14]), stod(tokens[15]));
        else if (tokens[0] == "LIGHT")
            scene.lights.emplace_back(tokens[1], Vec(stod(tokens[2]), stod(tokens[3]), stod(tokens[4])),
                                       Vec(stod(tokens[5]), stod(tokens[6]), stod(tokens[7])));
        else if (tokens[0] == "BACK") scene.bgColour = Vec(stod(tokens[1]), stod(tokens[2]), stod(tokens[3]));
        else if (tokens[0] == "AMBIENT") scene.ambientColour = Vec(stod(tokens[1]), stod(tokens[2]), stod(tokens[3]));
        else if (tokens[0] == "OUTPUT") scene.outputName = tokens[1].substr(4);
    }
    return scene;
}

// Output in P6 format, a binary file containing:
// P6
// ncolumns nrows
// Max colour value
// colours in binary format thus unreadable
void save_imageP6(unsigned int Width, unsigned int Height, char* fname,unsigned char* pixels) {
    FILE *fp;
    const unsigned int maxVal=255;

    printf("Saving image %s: %d x %d\n", fname,Width,Height);
    fp = fopen(fname,"wb");
    if (!fp) {
          printf("Unable to open file '%s'\n",fname);
          return;
    }
    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", Width, Height);
    fprintf(fp, "%d\n", maxVal);

    for(unsigned int j = 0; j < Height; j++) {
            fwrite(&pixels[j*Width*3], 3,Width,fp);
    }

    fclose(fp);
  }

  // Output in P3 format, a text file containing:
  // P3
  // ncolumns nrows
  // Max colour value (for us, and usually 255)
  // r1 g1 b1 r2 g2 b2 .....
  void save_imageP3(unsigned int Width, unsigned int Height, char* fname,unsigned char* pixels) {
      FILE *fp;
      const signed char maxVal=255;

      printf("Saving image %s: %d x %d\n", fname,Width,Height);
      fp = fopen(fname,"w");
      if (!fp) {
          printf("Unable to open file '%s'\n",fname);
          return;
      }
      fprintf(fp, "P3\n");
      fprintf(fp, "%d %d\n", Width, Height);
      fprintf(fp, "%d\n", maxVal);

      unsigned int k = 0 ;
      for(unsigned int j = 0; j < Height; j++) {

          for( unsigned int i = 0 ; i < Width; i++)
          {
              fprintf(fp," %d %d %d", pixels[k],pixels[k+1],pixels[k+2]) ;
              k = k + 3 ;
          }
          fprintf(fp,"\n") ;
      }
      fclose(fp);
  }

// Compute the direction of a ray for a specific pixel
Vec getPixelDirection(unsigned int row, unsigned int col, const Scene& scene) {
    double v = -scene.right + (2.0 * scene.right * row) / scene.height;
    double u = -scene.top + (2.0 * scene.top * col) / scene.width;
    return Vec(u, v, -scene.near).normalize();
}

// Render the entire scene by tracing rays for each pixel
vector<vector<Vec>> renderScene(const Scene& scene) {
    Vec eye(0, 0, 0);
    vector<vector<Vec>> pixels(scene.height, vector<Vec>(scene.width));

    for (unsigned int row = 0; row < scene.height; ++row) {
        for (unsigned int col = 0; col < scene.width; ++col) {
            Vec dir = getPixelDirection(row, col, scene); 
            // Create ray starting from the camera and compute
            Ray ray(eye, dir, 0); 
            pixels[row][col] = computeRay(ray, scene);
        }
    }

    return pixels; // Return the rendered pixel colors
}

// Convert the 2D vector of pixel colors into a 1D pixel buffer for image saving
unsigned char* convertToPixelBuffer(const vector<vector<Vec>>& pixelValues, const Scene& scene) {
    unsigned int w = scene.width, h = scene.height;
    auto* buffer = new unsigned char[w * h * 3]; // Allocate memory for the pixel buffer

    for (unsigned int row = 0; row < h; ++row) {
        unsigned int flipped = h - 1 - row;
        for (unsigned int col = 0; col < w; ++col) {
            unsigned int idx = (flipped * w + col) * 3;
            Vec color = pixelValues[row][col];
            buffer[idx]     = static_cast<unsigned char>(round(color.x * 255));
            buffer[idx + 1] = static_cast<unsigned char>(round(color.y * 255));
            buffer[idx + 2] = static_cast<unsigned char>(round(color.z * 255));
        }
    }

    return buffer;
}

const char* toCString(const string& str) {
    return str.c_str();
}


int main(int argc, char *argv[]) {
    Scene scene = readFile(argv[1]);
    auto pixelValues = renderScene(scene);
    unsigned char* pixels = convertToPixelBuffer(pixelValues, scene);
    save_imageP6(scene.width, scene.height, (char*)toCString(scene.outputName), pixels);
    delete[] pixels;
    return 0;
}
