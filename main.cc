#include <cmath>
#include <cstdio>
#include <optional>

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Rect.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>

namespace bh
{

constexpr float THETA = 0.5f;
constexpr float GRAVITY_CONSTANT = 0.1f;
constexpr float TIME_STEP = 1.0f;
constexpr float SOFTENING = 15.0f;

struct point_t
{
  float mass;             // 4-bytes
  sf::Vector2f position;  // 8-bytes
  sf::Vector2f velocity;  // 8-bytes
};

static inline bh::point_t
point_init (float mass, const sf::Vector2f &position,
            const sf::Vector2f &velocity = { 0.f, 0.f })
{
  return (bh::point_t){ .mass = mass,
                        .position = position,
                        .velocity = velocity };
}

struct quad_node_t
{
  alignas (8) float total_mass{ 0.f };        // 4-bytes
  sf::Vector2f center_of_mass{ 0.f, 0.f };    // 8-bytes
  sf::FloatRect boundary{};                   // 16-bytes
  std::optional<bh::point_t> point{};         // 24-bytes
  bh::quad_node_t *children[4]{ 0, 0, 0, 0 }; // 32-bytes
};

static inline bh::quad_node_t *
quad_node_init (const sf::FloatRect &boundary)
{
  auto *node = new bh::quad_node_t{};
  return node->boundary = boundary, node;
}

static inline void
quad_node_free (bh::quad_node_t *node)
{
  if (node == NULL)
    return;

  for (auto child : node->children)
    quad_node_free (child);

  delete node;
}

static inline bool
quad_node_is_leaf (const bh::quad_node_t &node)
{
  return std::all_of (
      node.children, node.children + 4,
      [] (const auto &child) { return child == NULL; });
}

static inline void
quad_node_subdivide (bh::quad_node_t *node)
{
  const sf::Vector2f center{
    node->boundary.left + node->boundary.width / 2,
    node->boundary.top + node->boundary.height / 2
  };

  const sf::FloatRect quadrants[4]
      = { { node->boundary.left, node->boundary.top,
            node->boundary.width / 2, node->boundary.height / 2 },
          { center.x, node->boundary.top, node->boundary.width / 2,
            node->boundary.height / 2 },
          { node->boundary.left, center.y, node->boundary.width / 2,
            node->boundary.height / 2 },
          { center.x, center.y, node->boundary.width / 2,
            node->boundary.height / 2 } };

  unsigned char index = 0;
  for (auto &child : node->children)
    child = bh::quad_node_init (quadrants[(index++) % 4]);
}

static inline void
quad_node_insert (bh::quad_node_t *node, const bh::point_t &point)
{
  if (!node->boundary.contains (point.position))
    return;

  if (bh::quad_node_is_leaf (*node))
    {
      if (!node->point.has_value ())
        return node->point = point, (void)0;

      bh::quad_node_subdivide (node);

      const bh::point_t save = node->point.value ();
      node->point.reset ();

      for (auto child : node->children)
        bh::quad_node_insert (child, save);
    }

  for (auto child : node->children)
    bh::quad_node_insert (child, point);
}

static inline void
quad_node_compute_mass (bh::quad_node_t *node)
{
  if (bh::quad_node_is_leaf (*node))
    {
      if (node->point.has_value ())
        {
          node->center_of_mass = node->point->position;
          node->total_mass = node->point->mass;
        }

      return;
    }

  node->center_of_mass = { 0.f, 0.f };
  node->total_mass = 0;

  for (auto child : node->children)
    {
      bh::quad_node_compute_mass (child);
      node->total_mass += child->total_mass;
      node->center_of_mass
          += child->center_of_mass * child->total_mass;
    }

  if (node->total_mass > 0)
    node->center_of_mass /= node->total_mass;
}

static inline void
quad_node_compute_force (const quad_node_t &node, point_t *point)
{
  if (node.total_mass == 0 || point->position == node.center_of_mass)
    return;

  const sf::Vector2f direction
      = node.center_of_mass - point->position;
  const float distance = std::sqrt (direction.x * direction.x
                                    + direction.y * direction.y
                                    + bh::SOFTENING * bh::SOFTENING);

  const float ratio = node.boundary.width / distance;
  if (bh::quad_node_is_leaf (node) || ratio < THETA)
    {
      const float force
          = bh::GRAVITY_CONSTANT * node.total_mass * point->mass
            / (distance * distance + bh::SOFTENING * bh::SOFTENING);
      const sf::Vector2f acceleration
          = direction / distance * force / point->mass;
      point->velocity += acceleration * bh::TIME_STEP;
    }
  else
    {
      for (auto child : node.children)
        bh::quad_node_compute_force (*child, point);
    }
}

}

int
main ()
{
  sf::RenderWindow window{sf::VideoMode{800, 800}, "Barnes-Hut Simulation", sf::Style::Default, sf::ContextSettings{24, 8, 8}};
  window.setFramerateLimit(60);

  sf::VertexArray vao{sf::Points};

  std::vector<bh::point_t> points{};
  for (int i = 0; i < 16000; ++i) {
    float x = static_cast<float>(std::rand() % 800);
    float y = static_cast<float>(std::rand() % 800);
    float dx = 400 - x, dy = 400 - y;
    float angle = atan2f(dy, dx) - M_PI / 2;

    points.emplace_back(bh::point_init(1.0f, {x, y}, {cosf(angle), sinf(angle)}));
  }

  sf::Clock delta;
  float fps_avg = 0;
  int frame = 0;
  while (window.isOpen())
    {
      const float dt = delta.restart().asSeconds();

      sf::Event event{};
      while (window.pollEvent(event))
        {
          if (event.type == sf::Event::Closed)
            window.close();
        }

      bh::quad_node_t *root = bh::quad_node_init({0, 0, 800, 800});
      for (size_t i = 0; i < points.size(); ++i)
        bh::quad_node_insert(root, points[i]);
      bh::quad_node_compute_mass(root);

#pragma omp parallel for
      for (size_t i = 0; i < points.size(); ++i)
        {
          bh::quad_node_compute_force(*root, &points[i]);
          points[i].position += points[i].velocity * bh::TIME_STEP;
        }

      window.clear({10, 10, 10});

      vao.clear();
      for (size_t i = 0; i < points.size(); ++i) {
        vao.append({points[i].position, {128, 148, 148}});
      }

      window.draw(vao);
      window.display();

      bh::quad_node_free(root);

      fps_avg += (frame++ == 0) ? 0 : 1.0f / dt;
      printf("%.2f (%.2f)\n", 1.0f / dt, fps_avg / frame);
    }

  return 0;
}

