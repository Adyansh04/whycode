#ifndef WHYCON_MARKER_GEOMETRY_HPP
#define WHYCON_MARKER_GEOMETRY_HPP

#include <optional>

namespace whycon
{

/**
 * @brief The shape checks' expectations for a marker drawn by whycode_gen, derived once at startup.
 *
 * Every WhyCode bit is one black and one white segment, so half of the teeth band is black
 * whatever the ID and the tooth count: the black and white areas depend on the outer and centre
 * diameters alone. The teeth do move the white centre's bounding box off the ring's, more the
 * fewer they are. Circularity, eccentricity and the other tolerances do not depend on the design.
 */
class MarkerGeometry
{
public:
    /**
     * @brief Describes one marker design.
     * @param outer_diameter Outer diameter of the black ring (m).
     * @param teeth Code bits; 0 for a WhyCon marker, which has no teeth.
     * @param centre_diameter Printed white centre, teeth included (m); whycode_gen's if empty.
     */
    MarkerGeometry(double outer_diameter, int teeth, std::optional<double> centre_diameter) noexcept;

    /**
     * @brief Printed white centre, teeth included.
     * @return Diameter (m).
     */
    double centreDiameter() const noexcept { return centre_diameter_; }

    /**
     * @brief Checks the centre against the design.
     * @return True inside the ring and, with teeth, outside the white disc they surround.
     */
    bool valid() const noexcept;

    /**
     * @brief The disc with the marker's white area, which the area checks expect.
     * @return Diameter (m).
     */
    double whiteAreaDiameter() const noexcept;

    /**
     * @brief How far the white centre's bounding box may sit off the ring's.
     * @return Offset over the ring's bounding box: the teeth's widest, plus perspective's.
     */
    double centreToleranceRatio() const noexcept;

private:
    // whycode_gen's radii over the outer radius (its canvas units / 700)
    static constexpr double WHYCODE_CENTRE = 420.0 / 700.0;  // Outer edge of the teeth band
    static constexpr double TEETH_INNER    = 240.0 / 700.0;  // White disc inside the teeth
    static constexpr double WHYCON_CENTRE  = 287.0 / 700.0;
    // Perspective moves the white centre's box about 0.03 of the ring's box at most, for markers up
    // to a quarter of the focal length across (a 0.2 m marker at 0.8 m on the Isaac camera)
    static constexpr double PERSPECTIVE_TOLERANCE = 0.03;

    double outer_diameter_;
    int    teeth_;
    double centre_diameter_;  // Printed white centre, teeth included (m)
};

}  // namespace whycon

#endif  // WHYCON_MARKER_GEOMETRY_HPP
