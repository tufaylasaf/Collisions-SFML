#include <iostream>
#include "src/body.cpp"
#include "src/solver.cpp"
#include <SFML/Graphics.hpp>

int main()
{
    const uint64_t WIDTH = 1280;
    const uint64_t HEIGHT = 720;

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "SFML Grid and Bodies");

    Solver solver = Solver(sf::Vector2f(WIDTH, HEIGHT), 475, 2500);

    sf::Clock clock;

    int frameCount = 0;
    float timeElapsed = 0.0f;
    float spawnInterval = 0.05f;
    float timeSinceLastSpawn = 0.0f;
    float avgFPS = 0.0f;

    sf::CircleShape circle{1.0f};
    circle.setPointCount(16);
    circle.setOrigin(1.0f, 1.0f);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dt = clock.restart().asSeconds();

        timeSinceLastSpawn += dt;
        if (avgFPS >= 60)
        {
            solver.spawnBodyFromCenter(spawnInterval, timeSinceLastSpawn, dt);
        }
        solver.update(1 / 30.0f, window);

        window.clear(sf::Color::Black);
        solver.spatialGrid.renderGrid(window);

        // Render bodies
        for (Body &b : solver.bodies)
        {
            circle.setPosition(b.position);
            circle.setScale(b.radius, b.radius);
            circle.setFillColor(b.color);
            window.draw(circle);
        }

        window.display();

        frameCount++;
        timeElapsed += dt;

        if (timeElapsed >= 1.0f)
        {
            avgFPS = frameCount / timeElapsed;
            std::cout << "Average FPS: " << avgFPS << std::endl;
            std::cout << "Bodies: " << solver.bodies.size() << std::endl;

            frameCount = 0;
            timeElapsed = 0.0f;
        }
    }

    return 0;
}