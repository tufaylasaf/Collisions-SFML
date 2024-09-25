#include "body.cpp"
#include <vector>
#include <map>
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

    // Spatial Hashing Variables
    float grid_size = 100.0f; // Increase grid size
    uint32_t grid_width, grid_height;
    std::map<std::pair<int, int>, std::vector<Body *>> grid;

    Solver() = default;

    Solver(sf::Vector2f c, float cr, uint32_t grid_w, uint32_t grid_h) : center(c * 0.5f), constRadius(cr),
                                                                         grid_width(grid_w), grid_height(grid_h)
    {
    }

    Body &addBody(float r, sf::Color c, sf::Vector2f p)
    {
        return bodies.emplace_back(Body(r, c, p));
    }

    // Generate distinct color based on grid position
    sf::Color getGridColor(int grid_x, int grid_y)
    {
        // Simple hash-based color generation for distinct colors per grid
        uint32_t hash_value = static_cast<uint32_t>(grid_x * 73856093 ^ grid_y * 19349663);
        uint8_t r = (hash_value & 0xFF0000) >> 16;
        uint8_t g = (hash_value & 0x00FF00) >> 8;
        uint8_t b = (hash_value & 0x0000FF);

        return sf::Color(r, g, b);
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

        // Clear grid for the next update
        grid.clear();

        for (Body &b : bodies)
        {
            // Compute grid coordinates based on body position
            int grid_x = static_cast<int>(b.position.x / grid_size);
            int grid_y = static_cast<int>(b.position.y / grid_size);

            // Add body pointer to the corresponding grid cell
            grid[{grid_x, grid_y}].push_back(&b);

            // Assign a color based on the grid cell
            b.color = getGridColor(grid_x, grid_y);
        }

        for (uint64_t i{subSteps}; i--;)
        {
            applyGravity(subDt);
            checkCollision(subDt);
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
            const float dist = (v.x * v.x + v.y * v.y);
            if (dist > (constRadius - b.radius) * (constRadius - b.radius))
            {
                float dist2 = sqrt(dist);
                const sf::Vector2f n = v / dist2;
                b.position = center - n * (constRadius - b.radius);
            }
        }
    };

    void checkCollision(float dt)
    {
        // Iterate over all grid cells
        for (auto &[cell, bodyList] : grid)
        {
            // Retrieve grid cell coordinates
            int grid_x = cell.first;
            int grid_y = cell.second;

            // Check for collisions within the same cell
            for (size_t i = 0; i < bodyList.size(); ++i)
            {
                Body &b1 = *bodyList[i];

                for (size_t j = i + 1; j < bodyList.size(); ++j)
                {
                    Body &b2 = *bodyList[j];
                    resolveCollision(b1, b2);
                }
            }

            // Check for collisions with neighboring cells
            for (int offset_x = -1; offset_x <= 1; ++offset_x)
            {
                for (int offset_y = -1; offset_y <= 1; ++offset_y)
                {
                    // Skip the current cell
                    if (offset_x == 0 && offset_y == 0)
                        continue;

                    std::pair<int, int> neighbor_cell = {grid_x + offset_x, grid_y + offset_y};

                    // If neighbor cell exists in the grid
                    if (grid.find(neighbor_cell) != grid.end())
                    {
                        std::vector<Body *> &neighbor_bodies = grid[neighbor_cell];

                        // Check for collisions between bodies in the current cell and neighboring bodies
                        for (Body *b1 : bodyList)
                        {
                            for (Body *b2 : neighbor_bodies)
                            {
                                resolveCollision(*b1, *b2);
                            }
                        }
                    }
                }
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

    void renderGrid(sf::RenderWindow &window)
    {
        // Draw grid lines
        for (uint32_t i = 0; i <= grid_width; ++i)
        {
            sf::RectangleShape line(sf::Vector2f(1.0f, window.getSize().y));
            line.setPosition(i * grid_size, 0);
            line.setFillColor(sf::Color(100, 100, 100));
            window.draw(line);
        }

        for (uint32_t j = 0; j <= grid_height; ++j)
        {
            sf::RectangleShape line(sf::Vector2f(window.getSize().x, 1.0f));
            line.setPosition(0, j * grid_size);
            line.setFillColor(sf::Color(100, 100, 100));
            window.draw(line);
        }
    }
};
