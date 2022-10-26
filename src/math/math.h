namespace math {

class Point2d {
public:
    //! Constructor which takes x- and y-coordinates.
    constexpr Point2d(const double x, const double y) noexcept : x_(x), y_(y) {}

    //! Constructor returning the zero vector.
    constexpr Point2d() noexcept : Point2d(0, 0) {}

    //! Getter for x component
    double x() const { return x_; }

    //! Getter for y component
    double y() const { return y_; }

    //! Setter for x component
    void set_x(const double x) { x_ = x; }

    //! Setter for y component
    void set_y(const double y) { y_ = y; }

    //! Sums two Point2d
    Point2d operator+(const Point2d &other) const;

    //! Subtracts two Point2d
    Point2d operator-(const Point2d &other) const;

    //! Multiplies Point2d by a scalar
    Point2d operator*(const double ratio) const;

    //! Divides Point2d by a scalar
    Point2d operator/(const double ratio) const;

    //! Sums another Point2d to the current one
    Point2d &operator+=(const Point2d &other);

    //! Subtracts another Point2d to the current one
    Point2d &operator-=(const Point2d &other);

    //! Multiplies this Point2d by a scalar
    Point2d &operator*=(const double ratio);

    //! Divides this Point2d by a scalar
    Point2d &operator/=(const double ratio);

    //! Compares two Point2d
    bool operator==(const Point2d &other) const;

protected:
    double x_ = 0.0;
    double y_ = 0.0;
};

} // math
