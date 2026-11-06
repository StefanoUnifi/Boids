//
// Created by stefa on 05/11/2026.
//

#ifndef BOIDS_BOID_H
#define BOIDS_BOID_H
#include <vector>

constexpr float MIN_SPEED = 3.0f;
constexpr float MAX_SPEED = 6.0f;
constexpr float PROTECTED_RANGE = 8.0f;
constexpr float VISION_RANGE = 40.0f;
constexpr float AVOID_FACTOR = 0.05f;
constexpr float MATCHING_FACTOR = 0.05f;
constexpr float CENTERING_FACTOR = 0.0005f;

//struct ausiliaria per avere una posizione in 2D (x,y) e per avere tipi di ritorno
struct Vettore2D {
    float x;
    float y;
};

class Boid {
    public:
        explicit Boid(float x, float y, float vx, float vy);
        Boid() = default;

        //getters e setters
        float getX() const;
        float getY() const;
        float getVx() const;
        float getVy() const;

        void setX(float x);
        void setY(float y);
        void setVx(float vx);
        void setVy(float vy);

        //regole dei boids
        Vettore2D separation(const std::vector<Boid>& allBoids) const;
        Vettore2D alignment(const std::vector<Boid>& allBoids) const;
        Vettore2D cohesion(const std::vector<Boid>& allBoids) const;

        //funzioni ausiliarie
        float distance(const Boid& otherBoid) const;

    private:
        float x, y, vx, vy; //vx, vy = velocità su x ed y (che sono sempre comprese tra MIN_SPEED e MAX_SPEED)
};

#endif //BOIDS_BOID_H
