#pragma once

#include <string>

namespace robot_slamd {

enum class ScalarTofReturnKind {
    Unspecified,
    Invalid,
    Hit,
    NoReturn
};

inline std::string to_string(ScalarTofReturnKind kind) {
    switch (kind) {
    case ScalarTofReturnKind::Unspecified:
        return "unspecified";
    case ScalarTofReturnKind::Invalid:
        return "invalid";
    case ScalarTofReturnKind::Hit:
        return "hit";
    case ScalarTofReturnKind::NoReturn:
        return "no_return";
    }
    return "unknown";
}

} // namespace robot_slamd
