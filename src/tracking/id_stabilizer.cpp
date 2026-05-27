#include "whycode/tracking/id_stabilizer.hpp"

#include <algorithm>

#include "whycode/utils/whycon_config.h"

namespace whycon
{

IDStabilizer::IDStabilizer(int switch_threshold)
  : switch_threshold_(std::max(1, switch_threshold))
{
    history_.reserve(INITIAL_CAPACITY);
    WHYCON_INFO("[IDStabilizer] Initialized with switch threshold: " << switch_threshold_);
}

int IDStabilizer::stabilize(int tracking_id, int current_decoded_id)
{
    // Lookup/insertion
    auto [it, inserted] = history_.try_emplace(tracking_id);

    // Case 1: This is a new track
    if (inserted)
    {
        // Only initialize if the first detection is valid
        if (isValidID(current_decoded_id))
        {
            WHYCON_INFO(
                "[IDStabilizer] Initializing new track " << tracking_id << " with ID "
                                                         << current_decoded_id);
            it->second = IDHistory(current_decoded_id, switch_threshold_);
            return current_decoded_id;
        }
        // Otherwise, wait for a valid ID
        return INVALID_ID;
    }

    // Case 2: This is an existing track
    IDHistory& history = it->second;

    // If the new detection matches the locked ID, reset its confidence
    if (current_decoded_id == history.locked_id)
    {
        updateConfidenceForMatch(history);
    }
    // If we get a valid but different ID, decrease confidence
    else if (isValidID(current_decoded_id))
    {
        decrementConfidenceForMismatch(history, tracking_id, current_decoded_id);

        // If confidence has run out, switch the locked ID to the new one
        if (shouldSwitchID(history))
        {
            switchLockedID(history, current_decoded_id, tracking_id);
        }
    }

    // Always return the currently locked-in ID
    return history.locked_id;
}

inline bool IDStabilizer::isValidID(int id) const noexcept { return id != INVALID_ID; }

inline void IDStabilizer::updateConfidenceForMatch(IDHistory& history) const noexcept
{
    history.confidence_counter = switch_threshold_;
}

inline void IDStabilizer::decrementConfidenceForMismatch(
    IDHistory& history, int tracking_id, int current_decoded_id) const
{
    history.confidence_counter--;
    WHYCON_DEBUG(
        "[IDStabilizer] Mismatch for track " << tracking_id << ". Locked: " << history.locked_id
                                             << ", Detected: " << current_decoded_id
                                             << ". Confidence now: " << history.confidence_counter);
}

inline bool IDStabilizer::shouldSwitchID(const IDHistory& history) const noexcept
{
    return history.confidence_counter <= 0;
}

inline void IDStabilizer::switchLockedID(IDHistory& history, int new_id, int tracking_id) const
{
    WHYCON_INFO(
        "[IDStabilizer] Track " << tracking_id << " switched from ID " << history.locked_id
                                << " to " << new_id);
    history.locked_id          = new_id;
    history.confidence_counter = switch_threshold_;
}

void IDStabilizer::removeTrack(int tracking_id)
{
    if (history_.erase(tracking_id) > 0)
    {
        WHYCON_INFO("[IDStabilizer] Removing history for lost track ID: " << tracking_id);
    }
}

void IDStabilizer::removeTracks(const std::vector<int>& tracking_ids)
{
    for (int id : tracking_ids)
    {
        removeTrack(id);
    }
}

}  // namespace whycon