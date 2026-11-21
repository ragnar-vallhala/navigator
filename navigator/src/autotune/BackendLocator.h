#pragma once
// Where the autotune backend (the `vayu_sitl_rtos` executable) lives.
//
// An installed app has no vayu checkout to walk up to, so the backend must be
// found beside the binary first; the repo build trees stay as the development
// fallback. The order is deliberate and tested (tst_autotune_backend_locator):
//
//   1. $VAYU_SITL_RTOS            explicit override (a file path)
//   2. <appDir>/vayu_sitl_rtos    how the user-level install ships it
//   3. <appDir>/lib/vayu_sitl_rtos
//   4. <appDir>/../lib/vayu_sitl_rtos
//   5. <repo>/build_sitl_rtos/vayu_sitl_rtos
//   6. <repo>/sim/host/build_sitl_rtos/vayu_sitl_rtos
//
// A candidate counts only if it is an existing regular file that is executable.
#include <QString>
#include <QStringList>

namespace autotune {

struct BackendLookup {
  QString found;       // first usable candidate, empty if none
  QStringList tried;   // every candidate checked, in order (for the error text)
};

// Candidates in lookup order. envOverride may be empty (then step 1 is absent).
QStringList backendCandidates(const QString &envOverride,
                              const QString &appDir, const QString &repoRoot);

BackendLookup findRtosBackend(const QString &envOverride,
                              const QString &appDir, const QString &repoRoot);

// Human-readable "not found" text naming every path tried plus how to fix it.
QString backendNotFoundMessage(const QStringList &tried);

} // namespace autotune
