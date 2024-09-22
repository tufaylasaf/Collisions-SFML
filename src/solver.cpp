#include "body.cpp"
#include <vector>
#include <math.h>
#include <numbers>
#include <random>

struct Solver
{
    std::vector<Body> bodies;
    sf::Vector2f g = sf::Vector2f(0, 1000.0f);
    sf::Vector2f center;
    float constRadius;
    float time;
    static constexpr double PI = 3.14159265358979323846;
    float speed = 147.0f;
    const uint64_t subSteps = 8;

    Solver() = default;

    Solver(sf::Vector2f c, float cr) : center(c * 0.5f), constRadius(cr)
    {
    }

    Body &addBody(float r, sf::Color c, sf::Vector2f p)
    {
        return bodies.emplace_back(Body(r, c, p));
    }

    static sf::Color getRainbow(float t)
    {
        const float r = sin(t);
        const float g = sin(t + 0.33f * 2.0f * PI);
        const float b = sin(t + 0.66f * 2.0f * PI);
        return {static_cast<uint8_t>(255.0f * r * r),
                static_cast<uint8_t>(255.0f * g * g),
                static_cast<uint8_t>(255.0f * b * b)};
    }

    void spawnBodyFromCenter(float spawnInterval, float &timeSinceLastSpawn, float dt)
    {
        if (timeSinceLastSpawn >= spawnInterval)
        {

            std::random_device rd;

            std::mt19937 gen(rd());

            std::uniform_real_distribution<> dis(8.0f, 18.0f);

            Body &b = addBody(dis(gen), getRainbow(time), center + sf::Vector2f(0, -center.y * 0.4f));
            float angle = sin(time / 4) + PI * 0.5f;
            b.setVelocity(speed * sf::Vector2f{cos(angle), sin(angle)}, dt);

            timeSinceLastSpawn = 0.0f;
        }
    }

    void update(float dt)
    {
        time += dt;
        float subDt = dt / static_cast<float>(subSteps);

        for (uint64_t i{subSteps}; i--;)
        {
            applyGravity(subDt);
            checkCollison(subDt);
            applyConstraint();
            updateBodies(subDt);
        }
    }

    void updateBodies(float dt)
    {
        for (Body &b : bodies)
        {
            b.update(dt);
        }
    }

    void applyGravity(float dt)
    {
        for (Body &b : bodies)
        {
            b.applyForce(g * dt);
        }
    }

    void applyConstraint()
    {
        for (Body &b : bodies)
        {
            const sf::Vector2f v = center - b.position;
            const float dist = sqrt(v.x * v.x + v.y * v.y);
            if (dist > (constRadius - b.radius))
            {
                const sf::Vector2f n = v / dist;
                b.position = center - n * (constRadius - b.radius);
            }
        }
    };

    void checkCollison(float dt)
    {
        const uint64_t count = bodies.size();
        for (uint64_t i{0}; i < count; ++i)
        {
            Body &b1 = bodies[i];

            for (uint64_t k{i + 1}; k < count; ++k)
            {
                Body &b2 = bodies[k];
                const sf::Vector2f v = b1.position - b2.position;
                const float dist2 = v.x * v.x + v.y * v.y;
                const float min_dist = b1.radius + b2.radius;

                if (dist2 < min_dist * min_dist)
                {
                    const float dist = sqrt(dist2);
                    const sf::Vector2f n = v / dist;
                    const float mass_ratio_1 = b1.radius / (b1.radius + b2.radius);
                    const float mass_ratio_2 = b2.radius / (b1.radius + b2.radius);
                    const float delta = 0.5f * 0.75f * (dist - min_dist);

                    b1.position -= n * (mass_ratio_2 * delta);
                    b2.position += n * (mass_ratio_1 * delta);
                }
            }
        }
    }
};
