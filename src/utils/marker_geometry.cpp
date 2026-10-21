#include "whycode/utils/marker_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace whycon
{

MarkerGeometry::MarkerGeometry(
    double outer_diameter, int teeth, std::optional<double> centre_diameter) noexcept
  : outer_diameter_(outer_diameter)
  , teeth_(teeth)
  , centre_diameter_(
        centre_diameter.value_or((teeth > 0 ? WHYCODE_CENTRE : WHYCON_CENTRE) * outer_diameter))
{}

bool MarkerGeometry::valid() const noexcept
{
    return outer_diameter_ > 0.0 && centre_diameter_ < outer_diameter_ &&
           centre_diameter_ > (teeth_ > 0 ? TEETH_INNER * outer_diameter_ : 0.0);
}

double MarkerGeometry::whiteAreaDiameter() const noexcept
{
    if (teeth_ == 0)
    {
        return centre_diameter_;
    }
    // The white disc inside the teeth and half of the band around it
    const double disc = TEETH_INNER * outer_diameter_;
    return std::sqrt(0.5 * (disc * disc + centre_diameter_ * centre_diameter_));
}

double MarkerGeometry::centreToleranceRatio() const noexcept
{
    if (teeth_ == 0)
    {
        return PERSPECTIVE_TOLERANCE;
    }
    // Manchester coding lets at most two black segments meet. Centred on an axis, they hold the
    // white back to the centre's edge pi/teeth off it, or to the inner disc, while the other side
    // may reach the edge: the box's centre moves half the difference, over two outer radii
    const double centre = centre_diameter_ / outer_diameter_;
    const double reach  = std::max(TEETH_INNER, centre * std::cos(M_PI / teeth_));
    return (centre - reach) / 4.0 + PERSPECTIVE_TOLERANCE;
}

}  // namespace whycon
