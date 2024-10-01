#include "body.cpp"
#include "spatialHashing.cpp"
#include <vector>
#include <map>
#include <cmath>

struct Solver
{
    std::vector<Body> bodies;
    sf::Vector2f g = sf::Vector2f(0, 1000.0f);
    sf::Vector2f center;
    float constRadius;
    float time;
    static constexpr double PI = 3.14159265358979323846;
    float speed = 100.0f;
    const uint64_t subSteps = 8;
    const float bRadius = 6.0f;

    // Spatial Grid
    float grid_size = bRadius * 2;
    SpatialGrid spatialGrid;

    Solver() = default;

    Solver(sf::Vector2f c, float cr, uint32_t maxNumBodies)
        : center(c * 0.5f), constRadius(cr), spatialGrid(grid_size, maxNumBodies)
    {
    }

    Body &addBody(float r, sf::Color c, sf::Vector2f p)
    {
        return bodies.emplace_back(Body(r, c, p));
    }

    static sf::Color getRainbow()
    {
        // Static variable to hold the current angle in degrees
        static float angle = 0.0f;

        // Increment the angle (you can change this value to control the speed)
        angle += 1.0f; // Increment by 1 degree

        // Wrap the angle around at 360 degrees
        if (angle >= 360.0f)
        {
            angle -= 360.0f; // Reset to avoid overflow
        }

        // Convert angle to radians for sine calculations
        float radians = angle * (PI / 180.0f);

        // Calculate RGB components based on sine of the angle
        const float r = sin(radians);
        const float g = sin(radians + 2.0f * PI / 3.0f); // Shifted for color separation
        const float b = sin(radians + 4.0f * PI / 3.0f); // Shifted for color separation

        // Return the color, ensuring values are in the range of 0-255
        return {static_cast<uint8_t>(255.0f * r * r),
                static_cast<uint8_t>(255.0f * g * g),
                static_cast<uint8_t>(255.0f * b * b)};
    }

    void spawnBodyFromCenter(float spawnInterval, float &timeSinceLastSpawn, float dt)
    {
        float angle = 2.25f * PI;
        if (timeSinceLastSpawn >= spawnInterval)
        {
            Body &b = addBody(bRadius, getRainbow(), center + sf::Vector2f(-center.x * 0.75f, -center.y * 0.75f));
            // float angle = sin(time / 4) + PI * 0.5f;
            b.setVelocity(speed * sf::Vector2f{static_cast<float>(cos(angle)), static_cast<float>(sin(angle))}, dt);

            timeSinceLastSpawn = 0.0f;
        }
    }

    void update(float dt, const sf::RenderWindow &window)
    {
        time += dt;
        float subDt = dt / static_cast<float>(subSteps);

        // Prepare the positions vector for the spatial grid
        std::vector<float> positions(bodies.size() * 2); // 2D positions (x, y)
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            positions[2 * i] = bodies[i].position.x;
            positions[2 * i + 1] = bodies[i].position.y;
        }

        // Create the spatial grid for this frame
        spatialGrid.create(positions);

        for (uint64_t i{subSteps}; i--;)
        {
            applyGravity(subDt);
            checkCollision(subDt);
            applyConstraint(window);
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

    void applyConstraint(const sf::RenderWindow &window)
    {
        // Get window dimensions
        const float windowWidth = window.getSize().x;
        const float windowHeight = window.getSize().y;

        // Apply damping factor to simulate energy loss during a bounce
        const float damping = 0.75f; // Adjust for more or less energy loss
        for (Body &b : bodies)
        {
            // Handle left and right boundaries
            if (b.position.x - b.radius < 0) // Left boundary
            {
                float overstep = b.radius - b.position.x;
                b.position.x = b.radius;                                                         // Snap position to the boundary
                b.position_last.x = b.position.x + (b.position_last.x - b.position.x) * damping; // Reverse position_last to reflect velocity change
            }
            else if (b.position.x + b.radius > windowWidth) // Right boundary
            {
                float overstep = b.position.x + b.radius - windowWidth;
                b.position.x = windowWidth - b.radius;                                           // Snap position to the boundary
                b.position_last.x = b.position.x + (b.position_last.x - b.position.x) * damping; // Reverse position_last
            }

            // Handle top and bottom boundaries
            if (b.position.y - b.radius < 0) // Top boundary
            {
                float overstep = b.radius - b.position.y;
                b.position.y = b.radius;                                                         // Snap position to the boundary
                b.position_last.y = b.position.y + (b.position_last.y - b.position.y) * damping; // Reverse position_last
            }
            else if (b.position.y + b.radius > windowHeight) // Bottom boundary
            {
                float overstep = b.position.y + b.radius - windowHeight;
                b.position.y = windowHeight - b.radius;                                          // Snap position to the boundary
                b.position_last.y = b.position.y + (b.position_last.y - b.position.y) * damping; // Reverse position_last
            }
        }
    }

    void checkCollision(float dt)
    {
        std::vector<float> positions(bodies.size() * 2); // 2D positions (x, y)
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            positions[2 * i] = bodies[i].position.x;
            positions[2 * i + 1] = bodies[i].position.y;
        }

        // Check collisions using the spatial grid
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            Body &b1 = bodies[i];

            // Query the grid for nearby objects
            spatialGrid.query(positions, i, b1.radius * 2.0f);

            // Iterate through the query results to check for collisions
            const std::vector<int> &results = spatialGrid.getQueryResults();
            for (int j = 0; j < spatialGrid.getQuerySize(); ++j)
            {
                if (i == results[j])
                    continue; // Skip checking the body with itself

                Body &b2 = bodies[results[j]];
                resolveCollision(b1, b2);
            }
        }
    }

    void resolveCollision(Body &b1, Body &b2)
    {
        // Quick bounding box test to avoid expensive calculations
        sf::Vector2f v = b1.position - b2.position;
        float dist2 = v.x * v.x + v.y * v.y;
        float min_dist = b1.radius + b2.radius;

        if (dist2 >= min_dist * min_dist)
            return; // No collision possible

        // Proceed with precise collision resolution
        float dist = sqrt(dist2);
        sf::Vector2f n = v / dist;
        float mass_ratio_1 = b1.radius / (b1.radius + b2.radius);
        float mass_ratio_2 = b2.radius / (b1.radius + b2.radius);
        float delta = 0.5f * 0.75f * (dist - min_dist);

        b1.position -= n * (mass_ratio_2 * delta);
        b2.position += n * (mass_ratio_1 * delta);
    }
};
