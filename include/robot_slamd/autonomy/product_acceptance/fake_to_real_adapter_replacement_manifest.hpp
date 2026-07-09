#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace robot_slamd {

struct FakeToRealAdapterReplacementItem {
    std::string fake_component;
    std::string real_component;
    std::string interface_name;
    bool replacement_required_before_live = true;
    std::string notes;
};

struct FakeToRealAdapterReplacementManifest {
    bool valid = false;
    std::vector<FakeToRealAdapterReplacementItem> items;
    std::string summary;
};

class FakeToRealAdapterReplacementManifestBuilder {
public:
    FakeToRealAdapterReplacementManifest build_default_manifest() const {
        FakeToRealAdapterReplacementManifest manifest;
        manifest.items.push_back(item(
            "RealSensorReplayPort",
            "RealTofImuWheelSensorPort",
            "RobotSlamSensorPort",
            "replace replay log with request-window real sensor adapter"));
        manifest.items.push_back(item(
            "FullAutonomousSlamFakeMotionPort",
            "RealSoftwareMotionPort",
            "RobotSlamMotionPort",
            "replace fake/shadow motion with reviewed real motion port"));
        manifest.items.push_back(item(
            "DeterministicSlamBackendBinding",
            "RealSlamBackendBinding",
            "SlamBackendBinding",
            "replace deterministic skeleton with production-reviewed backend"));
        manifest.items.push_back(item(
            "FakeMapStorage",
            "RealMapStorage",
            "MapStorage/Future",
            "replace metadata-only memory storage with real map storage"));
        manifest.items.push_back(item(
            "FakeRelocalizationBinding",
            "RealRelocalizationBinding",
            "RelocalizationBinding/Future",
            "replace fake confidence with reviewed real relocalization"));
        manifest.items.push_back(item(
            "Fake map artifact",
            "real occupancy/grid map artifact",
            "MapArtifact/Future",
            "fake metadata is not navigation map data"));
        manifest.items.push_back(item(
            "Fake pose candidate",
            "gated real pose candidate",
            "RelocalizationPose/Future",
            "no writeback until safety review"));
        manifest.valid = manifest.items.size() >= 7;
        manifest.summary = manifest.valid
                               ? "fake_to_real_adapter_manifest_valid"
                               : "fake_to_real_adapter_manifest_invalid";
        return manifest;
    }

    std::string write_text(
        const FakeToRealAdapterReplacementManifest &manifest) const {
        std::ostringstream o;
        o << "Fake To Real Adapter Replacement Manifest\n";
        o << "valid: " << (manifest.valid ? "true" : "false") << "\n";
        for (const auto &entry : manifest.items) {
            o << "- " << entry.fake_component << " -> "
              << entry.real_component << " [" << entry.interface_name << "]";
            o << " required_before_live="
              << (entry.replacement_required_before_live ? "true" : "false");
            o << " notes=" << entry.notes << "\n";
        }
        o << "summary: " << manifest.summary << "\n";
        return o.str();
    }

private:
    static FakeToRealAdapterReplacementItem item(
        const std::string &fake_component,
        const std::string &real_component,
        const std::string &interface_name,
        const std::string &notes) {
        FakeToRealAdapterReplacementItem entry;
        entry.fake_component = fake_component;
        entry.real_component = real_component;
        entry.interface_name = interface_name;
        entry.replacement_required_before_live = true;
        entry.notes = notes;
        return entry;
    }
};

} // namespace robot_slamd
