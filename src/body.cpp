#ifndef body
#define body

#include <SFML/Graphics.hpp>
#include <SFML/System/Vector2.hpp>
#include <cmath>

struct Body
{
    float radius;
    sf::Color color;
    sf::Vector2f position;
    sf::Vector2f position_last;
    sf::Vector2f acc;
    float maxSpeed = 125.0f;

    Body() = default;

    Body(float r, sf::Color c, sf::Vector2f p) : radius(r), color(c), position(p), position_last(p), acc(0, 0)
    {
    }

    void update(float dt)
    {
        sf::Vector2f displacement = position - position_last;

        sf::Vector2f velocity = displacement / dt;

        float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);

        if (speed > maxSpeed)
        {

            velocity = (velocity / speed) * maxSpeed;
            displacement = velocity * dt;
        }

        position_last = position;
        position = position + displacement + acc * (dt * dt);

        acc = sf::Vector2f(0, 0);
    }

    void applyForce(sf::Vector2f force)
    {
        acc += force;
    }

    void setVelocity(sf::Vector2f v, float dt)
    {
        position_last = position - (v * dt);
    }
};

#endif
