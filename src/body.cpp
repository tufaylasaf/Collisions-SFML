#ifndef body
#define body

#include <SFML/Graphics.hpp>
#include <SFML/System/Vector2.hpp>

struct Body
{
    float radius;
    sf::Color color;
    // sf::CircleShape shape;
    sf::Vector2f position;
    sf::Vector2f position_last;
    sf::Vector2f acc;

    Body() = default;

    Body(float r, sf::Color c, sf::Vector2f p) : radius(r), color(c), position(p), position_last(p), acc(0, 0)
    {
    }

    void update(float dt)
    {
        const sf::Vector2f displacement = position - position_last;

        position_last = position;
        position = position + displacement + acc * (dt * dt);

        acc = sf::Vector2f(0, 0);
    }

    void applyForce(sf::Vector2f force) { acc += force; }

    void setVelocity(sf::Vector2f v, float dt)
    {
        position_last = position - (v * dt);
    };
};

#endif
