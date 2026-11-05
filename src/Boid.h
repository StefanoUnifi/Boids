//
// Created by stefa on 05/11/2026.
//

#ifndef BOIDS_BOID_H
#define BOIDS_BOID_H

constexpr float MIN_SPEED = 3.0;
constexpr float MAX_SPEED = 6.0;
constexpr float PROTECTED_RANGE = 8.0;
constexpr float VISION_RANGE = 40.0;

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

        //funzioni dei boids
        void separation();  //TODO: valuta come implementare la separazione (meglio con vettore di boids)
        void alignment();   //TODO: stesso discorso
        void cohesion();    //TODO: stesso discorso

    private:
        float x, y, vx, vy; //vx, vy = velocità su x ed y (che sono sempre comprese tra MIN_SPEED e MAX_SPEED)
};

#endif //BOIDS_BOID_H
