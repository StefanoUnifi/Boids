#include <chrono>
#include <vector>
#include <iostream>
#include <random>

#include "Boid.h"

int main() {
    //parametri di inizializzazione per boids
    const int N_BOIDS = 400; //numero di boids
    std::vector<Boid> boids;
    boids.reserve(N_BOIDS);

    const int N_FRAMES = 1500; //numero di frame da simulare

    //definizione dei margini dello schermo effettivo
    const float SCREEN_WIDTH = 800.0f; //larghezza dello schermo
    const float SCREEN_HEIGHT = 600.0f; //altezza dello schermo
    ScreenBounds screenBounds(150.0f, SCREEN_WIDTH - 150.0f, 150.0f, SCREEN_HEIGHT - 150.0f);

    //per randomizzare le posizioni e le velocità iniziali dei boids
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis_x(150.0f, SCREEN_WIDTH - 150.0f);
    std::uniform_real_distribution<float> dis_y(150.0f, SCREEN_HEIGHT - 150.0f);
    std::uniform_real_distribution<float> dis_vel(-5.0f, 5.0f);

    //inizializza i boids con posizioni e velocità casuali
    for (int i = 0; i < N_BOIDS; i++){
        float x = dis_x(gen);
        float y = dis_y(gen);
        float vx = dis_vel(gen);
        float vy = dis_vel(gen);

        int scoutGroup = 0;
        float biasVal = 0.001f;
        //assegna pseudo-casualmente i boids a uno dei due gruppi di scout
        if (i < N_BOIDS * 0.1f) { //10% dei boids sono scout che vanno verso destra
            scoutGroup = 1;
        } else if (i < N_BOIDS * 0.2f) { //10% dei boids sono scout che vanno verso sinistra
            scoutGroup = 2; //gruppo di scout 2
        }
        boids.emplace_back(x, y, vx, vy, scoutGroup, biasVal);
    }

    std::vector <Boid> boids_copy = boids; //copia dei boids per l'aggiornamento

    //inizio simulazione per N_FRAMES
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < N_FRAMES; frame++) {
        //aggiorna i boids
        for (size_t i = 0; i < boids.size(); ++i) {
            boids_copy[i] = boids[i]; //copia lo stato attuale del boid
            boids_copy[i].updateOptimized(boids, screenBounds); //aggiorna il boid
        }
        boids = boids_copy; //aggiorna lo stato dei boids con la copia aggiornata
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

    std::cout << "Simulazione completata per " << N_FRAMES << " frame con " << N_BOIDS << " boids. " << std::endl;
    std::cout << "Tempo impiegato: " << elapsed.count() << " ms\n" << std::endl;

    return 0;
}