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
Vettore2D Boid::separation(const std::vector<Boid>& allBoids) const {
    float close_dx = 0.0f;
    float close_dy = 0.0f;

    for (const auto& otherBoid : allBoids) {
        if (this != &otherBoid) { //per evitare di confrontare il currentBoid con se stesso
            //Calcola la distanza tra attuale e altro boid per vedere se è nel PROTECTED_RANGE
            float dist = this->distance(otherBoid);
            if (dist > 0.0f && dist < PROTECTED_RANGE) {
                close_dx += this->getX() - otherBoid.getX();
                close_dy += this->getY() - otherBoid.getY();
            }
        }
    }
    //steer indica di quanto (in termini di velocità) deve sterzare il currentBoid rispetto a quelli nel PROTECTED_RANGE (se + vicino --> più potente la sterzata)
    Vettore2D steer = {0.0f, 0.0f};
    steer.x = close_dx * AVOID_FACTOR;
    steer.y = close_dy * AVOID_FACTOR;

    return steer;
}

//allineamento
Vettore2D Boid::alignment(const std::vector<Boid>& allBoids) const {
    float xvel_avg = 0.0f;
    float yvel_avg = 0.0f;
    int neighboring_boids = 0;

    for (const auto& otherBoid : allBoids) {
        if (this != &otherBoid) {
            float dist = this->distance(otherBoid);
            //Calcola la distanza tra attuale e altro boid per vedere se è nel VISION_RANGE
            if (dist > 0.0f && dist < VISION_RANGE) {
                xvel_avg += otherBoid.getVx();
                yvel_avg += otherBoid.getVy();
                neighboring_boids++;
            }
        }
    }

    Vettore2D align = {0.0f, 0.0f};
    if (neighboring_boids > 0) {
        xvel_avg /= neighboring_boids;
        yvel_avg /= neighboring_boids;

        align.x += (xvel_avg - this->getVx()) * MATCHING_FACTOR;
        align.y += (yvel_avg - this->getVy()) * MATCHING_FACTOR;
    }

    return align;
}

//coesione
Vettore2D Boid::cohesion(const std::vector<Boid>& allBoids) const {
    float xpos_avg = 0.0f;
    float ypos_avg = 0.0f;
    int neighboring_boids = 0;

    for (const auto& otherBoid : allBoids) {
        if (this != &otherBoid) {
            float dist = this->distance(otherBoid);
            if (dist > 0.0f && dist < VISION_RANGE) {
                xpos_avg += otherBoid.getX();
                ypos_avg += otherBoid.getY();
                neighboring_boids++;
            }
        }
    }

    Vettore2D cohesion = {0.0f, 0.0f};
    if (neighboring_boids > 0) {
        xpos_avg /= neighboring_boids;
        ypos_avg /= neighboring_boids;

        cohesion.x = (xpos_avg - this->getX()) * CENTERING_FACTOR;
        cohesion.y = (ypos_avg - this->getY()) * CENTERING_FACTOR;
    }

    return cohesion;
}