#ifndef ID_STABILIZER_HPP
#define ID_STABILIZER_HPP

#include <unordered_map>
#include <vector>

namespace whycon {

/**
 * @brief Stabilizes flickering WhyCode IDs for tracked markers using a hysteresis filter.
 *
 * This class maintains a "locked-in" ID for each track. A new ID must be
 * detected consecutively for a certain number of frames before it can
 * overwrite the existing locked-in ID. This prevents transient noise from
 * causing the output ID to flicker.
 */
class IDStabilizer {
  public:
    /**
     * @brief Constructs the ID stabilizer.
     * @param switch_threshold The number of consecutive mismatches required to switch the locked-in ID.
     */
    explicit IDStabilizer(int switch_threshold);

    /**
     * @brief Processes a new decoded ID and returns a stabilized version.
     * @param tracking_id The persistent tracking ID from the MarkerTracker.
     * @param current_decoded_id The raw decoded ID from the current frame (-1 if unreliable).
     * @return The stabilized ID.
     */
    int stabilize(int tracking_id, int current_decoded_id);

    /**
     * @brief Removes history for multiple tracks.
     * @param tracking_ids A vector of track IDs to remove.
     */
    void removeTracks(const std::vector<int>& tracking_ids);

  private:
    // A simple struct to hold the state for the hysteresis filter.

    static constexpr int    INVALID_ID         = -1;
    static constexpr int    INITIAL_CONFIDENCE = 0;
    static constexpr size_t INITIAL_CAPACITY   = 30;  // Pre-allocate for marker counts

    struct IDHistory {
        int locked_id;
        int confidence_counter;

        IDHistory() : locked_id(INVALID_ID), confidence_counter(INITIAL_CONFIDENCE) {}
        IDHistory(int id, int confidence) : locked_id(id), confidence_counter(confidence) {}
    };

    void        removeTrack(int tracking_id);
    inline bool isValidID(int id) const noexcept;
    inline void updateConfidenceForMatch(IDHistory& history) const noexcept;
    inline void decrementConfidenceForMismatch(IDHistory& history, int tracking_id, int current_decoded_id) const;
    inline bool shouldSwitchID(const IDHistory& history) const noexcept;
    inline void switchLockedID(IDHistory& history, int new_id, int tracking_id) const;

    std::unordered_map<int, IDHistory> history_;
    int                                switch_threshold_;
};

}  // namespace whycon

#endif  // ID_STABILIZER_HPP