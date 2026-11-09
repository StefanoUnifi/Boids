#include <chrono>
#include <vector>
#include <iostream>
#include <random>

#include "Boid.h"

int main() {
    //parametri di inizializzazione per boids
    const int N_BOIDS = 500; //numero di boids
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

    //versione modulare (con le 3 regole separate)
    std::vector <Boid> boids_mod = boids;
    std::vector <Boid> boids_mod_copy = boids_mod;

    auto start_mod_time = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < N_FRAMES; frame++) {
        //aggiorna i boids
        for (size_t i = 0; i < boids.size(); ++i) {
            boids_mod_copy[i] = boids_mod[i]; //copia lo stato attuale del boid
            boids_mod_copy[i].updateModular(boids_mod, screenBounds); //aggiorna il boid
        }
        boids_mod = boids_mod_copy; //aggiorna lo stato dei boids con la copia aggiornata
    }
    auto end_mod_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_mod = end_mod_time - start_mod_time;


    //versione ottimizzata (con le 3 regole combinate)
    std::vector <Boid> boids_opt = boids;
    std::vector <Boid> boids_opt_copy = boids_opt;

    auto start_opt_time = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < N_FRAMES; frame++) {
        for (size_t i = 0; i < boids.size(); ++i) {
            boids_opt_copy[i] = boids_opt[i];
            boids_opt_copy[i].updateOptimized(boids_opt, screenBounds);
        }
        boids_opt = boids_opt_copy;
    }
    auto end_opt_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_opt = end_opt_time - start_opt_time;

    std::cout << "Simulazione completata per " << N_FRAMES << " frame con " << N_BOIDS << " boids. " << std::endl;
    std::cout << "1. Tempo impiegato per versione MODULARE: " << elapsed_mod.count() << " ms\n" << std::endl;
    std::cout << "2. Tempo impiegato per versione OTTIMIZZATA: " << elapsed_opt.count() << " ms\n" << std::endl;

    return 0;
}