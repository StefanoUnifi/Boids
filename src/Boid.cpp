//
// Created by stefa on 05/11/2026.
//
#include "Boid.h"

#include <cmath>
#include <vector>

//costruttore
Boid::Boid(float x, float y, float vx, float vy) : x(x), y(y), vx(vx), vy(vy) {}

//getters e setter
float Boid::getX() const { return x; }
float Boid::getY() const { return y; }
float Boid::getVx() const { return vx; }
float Boid::getVy() const { return vy; }

void Boid::setX(float x) { this->x = x; }
void Boid::setY(float y) { this->y = y; }
void Boid::setVx(float vx) { this->vx = vx; }
void Boid::setVy(float vy) { this->vy = vy; }

float Boid::distance(const Boid& otherBoid) const {
    float dx = this->x - otherBoid.x;
    float dy = this->y - otherBoid.y;
    return std::sqrt(dx * dx + dy * dy);
}

//separazione
Posizione2D Boid::separation(const std::vector<Boid>& allBoids) const {
    float close_dx = 0.0f;
    float close_dy = 0.0f;

    for (const auto& otherBoid : allBoids) {
        if (this != &otherBoid) { //per evitare di confrontare il currentBoid con se stesso
            // Calcola la direzione opposta per allontanarsi
            float dist = this->distance(otherBoid);

            if (dist > 0.0f && dist < PROTECTED_RANGE) {
                close_dx += this->getX() - otherBoid.getX();
                close_dy += this->getY() - otherBoid.getY();
            }
        }
    }
    //steer indica di quanto (in termini di velocità) deve sterzare il currentBoid rispetto a quelli nel PROTECTED_RANGE (se + vicino --> più potente la sterzata)
    Posizione2D steer = {0.0f, 0.0f};
    steer.x = close_dx * AVOID_FACTOR;
    steer.y = close_dy * AVOID_FACTOR;

    return steer;
}
