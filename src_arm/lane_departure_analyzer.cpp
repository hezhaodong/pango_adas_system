#include "lane_departure_analyzer.h"

#include <algorithm>
#include <cmath>

namespace {
bool fitLeastSquares(const std::vector<LaneDepartureAnalyzer::Sample> &points,
                     LaneDepartureAnalyzer::Line &line)
{
    if (points.size() < 4) return false;
    double sumY = 0.0, sumX = 0.0;
    for (const auto &p : points) { sumY += p.y; sumX += p.x; }
    const double meanY = sumY / points.size();
    const double meanX = sumX / points.size();
    double numerator = 0.0, denominator = 0.0;
    for (const auto &p : points) {
        const double dy = p.y - meanY;
        numerator += dy * (p.x - meanX);
        denominator += dy * dy;
    }
    if (denominator < 1.0) return false;
    line.a = static_cast<float>(numerator / denominator);
    line.b = static_cast<float>(meanX - line.a * meanY);
    line.inliers = static_cast<int>(points.size());
    return std::isfinite(line.a) && std::isfinite(line.b);
}

}

void LaneDepartureAnalyzer::configure(const YoloV8Params &params)
{
    m_enabled = params.laneDepartureEnabled;
    m_cameraCenterRatio = std::clamp(params.cameraCenterRatio, 0.35f, 0.65f);
    m_warningThreshold = std::clamp(params.laneDepartureThreshold, 0.08f, 0.30f);
    m_clearThreshold = m_warningThreshold * 0.67f;
    m_confirmFrames = std::clamp(params.laneDepartureConfirmFrames, 1, 20);
    reset();
}

void LaneDepartureAnalyzer::reset()
{
    m_hasSmoothGeometry = false;
    m_candidateDirection = 0;
    m_candidateFrames = 0;
    m_alertDirection = 0;
    m_clearFrames = 0;
    m_missingFrames = 0;
    m_roadCandidateDirection = 0;
    m_roadCandidateFrames = 0;
    m_roadDirection = 0;
}

bool LaneDepartureAnalyzer::fitRobust(const std::vector<Sample> &points,
                                      int width, Line &line) const
{
    if (points.size() < 4) return false;
    const float residualLimit = std::max(16.0f, width * 0.025f);
    int bestCount = 0;
    float bestError = 1.0e30f;
    Line bestLine;

    // Deterministic RANSAC over all point pairs. There are normally only
    // 20-30 band samples, so this is cheap and resists jumps to adjacent lanes.
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            const float dy = points[j].y - points[i].y;
            if (std::fabs(dy) < 10.0f) continue;
            Line candidate;
            candidate.a = (points[j].x - points[i].x) / dy;
            candidate.b = points[i].x - candidate.a * points[i].y;
            int count = 0;
            float error = 0.0f;
            for (const Sample &point : points) {
                const float residual = std::fabs(lineX(candidate, point.y) - point.x);
                if (residual <= residualLimit) {
                    ++count;
                    error += residual;
                }
            }
            if (count > bestCount || (count == bestCount && error < bestError)) {
                bestCount = count;
                bestError = error;
                bestLine = candidate;
            }
        }
    }
    if (bestCount < 4) return false;

    std::vector<Sample> inliers;
    inliers.reserve(bestCount);
    for (const Sample &point : points) {
        if (std::fabs(lineX(bestLine, point.y) - point.x) <= residualLimit)
            inliers.push_back(point);
    }
    return fitLeastSquares(inliers, line);
}

bool LaneDepartureAnalyzer::fitAnchoredRoadCurve(
    const std::vector<Sample> &points, int width, int curveTopY, int anchorY,
    float anchorX, float yellowSlope, float &farOffsetPx,
    float &confidence) const
{
    farOffsetPx = 0.0f;
    confidence = 0.0f;
    if (points.size() < 4 || width <= 0 || anchorY <= curveTopY)
        return false;

    // The yellow line is already the application's near-field straight fit.
    // Build the thin red road curve as a constrained continuation of it:
    //
    //   x_red(y) = x_yellow(y) + D * u^2
    //   u = (anchorY - y) / (anchorY - curveTopY)
    //
    // At u=0 (the yellow line's top endpoint), red and yellow have exactly
    // the same x AND the same first derivative. D is therefore simply the
    // red-vs-yellow horizontal separation at curveTopY. Negative D means the
    // red lane center has bent left of the yellow extension; positive means
    // right. This directly implements the visual comparison used on screen.
    const float span = static_cast<float>(anchorY - curveTopY);
    const float residualLimit = std::max(12.0f, width * 0.015f);

    struct Observation {
        float u2;
        float residual;
    };
    std::vector<Observation> observations;
    observations.reserve(points.size());

    for (const Sample &point : points) {
        if (point.y < curveTopY || point.y > anchorY)
            continue;
        const float u = (anchorY - point.y) / span;
        // Very close to the shared start, u^2 is tiny and cannot constrain D.
        if (u < 0.16f || u > 1.05f)
            continue;
        const float yellowX = anchorX + yellowSlope * (point.y - anchorY);
        const float residual = point.x - yellowX;
        const float u2 = u * u;
        if (std::isfinite(residual) && std::isfinite(u2))
            observations.push_back({u2, residual});
    }
    if (observations.size() < 4)
        return false;

    // One-parameter deterministic RANSAC. Each observation proposes the far
    // offset D=residual/u^2; score that D against all other red-mask centers.
    int bestCount = 0;
    float bestError = 1.0e30f;
    float bestD = 0.0f;
    for (const Observation &seed : observations) {
        if (seed.u2 < 1.0e-4f) continue;
        const float candidateD = seed.residual / seed.u2;
        if (!std::isfinite(candidateD) || std::fabs(candidateD) > width * 0.8f)
            continue;

        int count = 0;
        float error = 0.0f;
        for (const Observation &obs : observations) {
            const float e = std::fabs(obs.residual - candidateD * obs.u2);
            if (e <= residualLimit) {
                ++count;
                error += e;
            }
        }
        if (count > bestCount || (count == bestCount && error < bestError)) {
            bestCount = count;
            bestError = error;
            bestD = candidateD;
        }
    }
    if (bestCount < 3)
        return false;

    // Least-squares refinement on the RANSAC inliers.
    double numerator = 0.0;
    double denominator = 0.0;
    float maxU2 = 0.0f;
    int inliers = 0;
    for (const Observation &obs : observations) {
        if (std::fabs(obs.residual - bestD * obs.u2) > residualLimit)
            continue;
        numerator += static_cast<double>(obs.u2) * obs.residual;
        denominator += static_cast<double>(obs.u2) * obs.u2;
        maxU2 = std::max(maxU2, obs.u2);
        ++inliers;
    }
    if (inliers < 3 || denominator < 1.0e-6 || maxU2 < 0.25f)
        return false;

    farOffsetPx = static_cast<float>(numerator / denominator);
    if (!std::isfinite(farOffsetPx) || std::fabs(farOffsetPx) > width * 0.8f)
        return false;

    const float inlierRatio = static_cast<float>(inliers) / observations.size();
    const float farCoverage = std::clamp(std::sqrt(maxU2), 0.0f, 1.0f);
    confidence = inlierRatio * farCoverage;
    return confidence >= 0.28f;
}

void LaneDepartureAnalyzer::update(YoloDetectResult &result)
{
    result.laneDepartureAlert = false;
    result.laneDepartureDirection = 0;
    result.roadDirection = m_roadDirection;
    result.roadDirectionRatio = 0.0f;
    result.roadDirectionConfidence = 0.0f;
    result.roadLeftCurvature = 0.0f;
    result.roadRightCurvature = 0.0f;
    result.roadLeftCurveValid = false;
    result.roadRightCurveValid = false;
    result.roadCurveTopY = 0;
    result.roadLeftCurveConfidence = 0.0f;
    result.roadRightCurveConfidence = 0.0f;
    result.roadTriangleLeftSourcePoints = 0;
    result.roadTriangleRightSourcePoints = 0;
    result.roadLeftVotePoints = 0;
    result.roadRightVotePoints = 0;
    result.roadStraightVotePoints = 0;
    result.laneLeftCenterXs.clear();
    result.laneLeftCenterYs.clear();
    result.laneRightCenterXs.clear();
    result.laneRightCenterYs.clear();
    if (!m_enabled || !result.hasSegmentation()) {
        reset();
        return;
    }

    const int width = result.segmentationWidth;
    const int height = result.segmentationHeight;
    const int cameraX = std::clamp(static_cast<int>(width * m_cameraCenterRatio),
                                   0, width - 1);
    const int scanTopY = static_cast<int>(height * 0.45f);
    // Evaluate/display the fitted geometry in the near field. Extrapolating
    // dashed or curved lanes up to the horizon often makes two valid fits
    // cross and incorrectly fail validation.
    const int topY = static_cast<int>(height * 0.62f);
    const int bottomY = static_cast<int>(height * 0.95f);
    const int bandHeight = std::max(8, height / 60);
    const int minSideGap = std::max(8, width / 80);
    std::vector<Sample> leftPoints, rightPoints;

    // Accumulate each horizontal band before choosing a boundary. This works
    // with dashed and thick YOLOPv2 lane masks that have many empty rows.
    std::vector<int> histogram(width);
    for (int y0 = scanTopY; y0 <= bottomY; y0 += bandHeight) {
        const int y1 = std::min(bottomY + 1, y0 + bandHeight);
        std::fill(histogram.begin(), histogram.end(), 0);
        for (int y = y0; y < y1; ++y) {
            const size_t row = static_cast<size_t>(y) * width;
            for (int x = 0; x < width; ++x)
                if (result.laneMask[row + x] != 0) ++histogram[x];
        }

        int bestLeft = -1, bestRight = width;
        int x = 0;
        while (x < width) {
            while (x < width && histogram[x] == 0) ++x;
            if (x >= width) break;
            int weightedX = 0, pixels = 0, gap = 0;
            while (x < width && gap <= 3) {
                if (histogram[x] > 0) {
                    weightedX += x * histogram[x];
                    pixels += histogram[x];
                    gap = 0;
                } else {
                    ++gap;
                }
                ++x;
            }
            if (pixels < 2) continue;
            const int center = weightedX / pixels;
            if (center < cameraX - minSideGap) bestLeft = std::max(bestLeft, center);
            if (center > cameraX + minSideGap) bestRight = std::min(bestRight, center);
        }
        const float sampleY = (y0 + y1 - 1) * 0.5f;
        // Yellow geometry is a near-field straight reference.  Do not let the
        // far curved section rotate that reference toward the bend.
        if (sampleY >= topY) {
            if (bestLeft >= 0)
                leftPoints.push_back({sampleY, static_cast<float>(bestLeft)});
            if (bestRight < width)
                rightPoints.push_back({sampleY, static_cast<float>(bestRight)});
        }
    }

    result.laneLeftSamples = static_cast<int>(leftPoints.size());
    result.laneRightSamples = static_cast<int>(rightPoints.size());

    // Keep the raw center samples before any straight/curve fit. These are the
    // actual centers of the thick lane-mask blobs selected in each scan band.
    // Drawing these directly makes it obvious whether the far-end merged mask
    // is pulling the selected center away from the desired lane boundary.
    result.laneLeftCenterXs.reserve(leftPoints.size());
    result.laneLeftCenterYs.reserve(leftPoints.size());
    for (const Sample &pt : leftPoints) {
        result.laneLeftCenterXs.push_back(
            std::clamp(static_cast<int>(std::lround(pt.x)), 0, width - 1));
        result.laneLeftCenterYs.push_back(
            std::clamp(static_cast<int>(std::lround(pt.y)), 0, height - 1));
    }
    result.laneRightCenterXs.reserve(rightPoints.size());
    result.laneRightCenterYs.reserve(rightPoints.size());
    for (const Sample &pt : rightPoints) {
        result.laneRightCenterXs.push_back(
            std::clamp(static_cast<int>(std::lround(pt.x)), 0, width - 1));
        result.laneRightCenterYs.push_back(
            std::clamp(static_cast<int>(std::lround(pt.y)), 0, height - 1));
    }

    Line leftLine, rightLine;
    const bool leftFit = fitRobust(leftPoints, width, leftLine);
    const bool rightFit = fitRobust(rightPoints, width, rightLine);
    bool valid = leftFit && rightFit;
    result.laneFitStatus = !leftFit ? 1 : !rightFit ? 2 : 0;
    float lt = 0, lb = 0, rt = 0, rb = 0;
    if (valid) {
        lt = lineX(leftLine, static_cast<float>(topY));
        lb = lineX(leftLine, static_cast<float>(bottomY));
        rt = lineX(rightLine, static_cast<float>(topY));
        rb = lineX(rightLine, static_cast<float>(bottomY));
        const float topWidth = rt - lt;
        const float bottomWidth = rb - lb;
        // Preserve candidate coordinates on failure so the display can report
        // the rejected widths during board-side calibration.
        result.laneGeometryTopY = topY;
        result.laneGeometryBottomY = bottomY;
        result.laneLeftTopX = static_cast<int>(std::lround(lt));
        result.laneLeftBottomX = static_cast<int>(std::lround(lb));
        result.laneRightTopX = static_cast<int>(std::lround(rt));
        result.laneRightBottomX = static_cast<int>(std::lround(rb));
        valid = lt < rt && lb < rb &&
                topWidth > width * 0.01f &&
                bottomWidth > width * 0.08f && bottomWidth < width * 1.10f &&
                topWidth < bottomWidth * 3.50f;
        if (!valid) result.laneFitStatus = 3;
    }

    if (!valid) {
        if (++m_missingFrames >= 5) {
            m_alertDirection = 0;
            m_candidateFrames = 0;
            m_hasSmoothGeometry = false;
        }
        result.cameraCenterX = cameraX;
        result.laneDepartureDirection = m_alertDirection;
        result.laneDepartureAlert = m_alertDirection != 0;
        return;
    }

    m_missingFrames = 0;
    if (!m_hasSmoothGeometry) {
        m_leftTop = lt; m_leftBottom = lb;
        m_rightTop = rt; m_rightBottom = rb;
        m_hasSmoothGeometry = true;
    } else {
        constexpr float alpha = 0.32f;
        m_leftTop += alpha * (lt - m_leftTop);
        m_leftBottom += alpha * (lb - m_leftBottom);
        m_rightTop += alpha * (rt - m_rightTop);
        m_rightBottom += alpha * (rb - m_rightBottom);
    }

    const float bottomWidth = m_rightBottom - m_leftBottom;
    const float laneCenter = (m_leftBottom + m_rightBottom) * 0.5f;
    const float offset = bottomWidth > 1.0f ? (cameraX - laneCenter) / bottomWidth : 0.0f;
    int candidate = 0;
    if (offset > m_warningThreshold) candidate = 1;
    else if (offset < -m_warningThreshold) candidate = -1;

    if (candidate != 0) {
        m_clearFrames = 0;
        if (candidate == m_candidateDirection) ++m_candidateFrames;
        else { m_candidateDirection = candidate; m_candidateFrames = 1; }
        if (m_candidateFrames >= m_confirmFrames) m_alertDirection = candidate;
    } else if (std::fabs(offset) < m_clearThreshold) {
        m_candidateDirection = 0;
        m_candidateFrames = 0;
        if (++m_clearFrames >= 2) m_alertDirection = 0;
    }

    // Triangle-ROI center extraction and road direction.
    //
    // Stage 1 above intentionally keeps the original yellow-line fit intact.
    // Stage 2 starts only after that fit is valid: the two yellow lines are
    // extrapolated upward and define a forward triangular/wedge ROI. We scan
    // that ROI at half the old vertical step, keep the left and right halves
    // completely independent, and select only the lane-mask component nearest
    // the corresponding yellow boundary.  Unlike the coarse fit pass, this
    // dense pass NEVER bridges zero-lane gaps, so a green/drivable separator
    // between two red lane components remains a hard separator.  If one red
    // blob is already continuous across the wedge centre, that band is marked
    // ambiguous and neither side is accepted; the two tracks are never joined.
    const float yellowSpan = static_cast<float>(bottomY - topY);
    const float leftYellowSlope = yellowSpan > 1.0f
        ? (m_leftBottom - m_leftTop) / yellowSpan : 0.0f;
    const float rightYellowSlope = yellowSpan > 1.0f
        ? (m_rightBottom - m_rightTop) / yellowSpan : 0.0f;
    const int denseBandHeight = std::max(4, bandHeight / 2);
    const int minWedgeWidth = std::max(20, width / 55);
    const int edgePad = std::max(1, width / 640);
    // Prefer the original direction region above the yellow-line top. Only
    // when that region contains no accepted red point at all, extend the
    // direction region downward to the upper quarter of the yellow segment.
    const int extendedDirectionAnchorY = topY + (bottomY - topY) / 4;

    std::vector<Sample> triangleLeftPoints;
    std::vector<Sample> triangleRightPoints;
    triangleLeftPoints.reserve((bottomY - scanTopY) / denseBandHeight + 2);
    triangleRightPoints.reserve((bottomY - scanTopY) / denseBandHeight + 2);

    int topLeftSourcePoints = 0, topRightSourcePoints = 0;
    int extendedLeftSourcePoints = 0, extendedRightSourcePoints = 0;
    long long topLeftSourcePixels = 0, topRightSourcePixels = 0;
    long long extendedLeftSourcePixels = 0, extendedRightSourcePixels = 0;
    int topAmbiguousMergedBands = 0, extendedAmbiguousMergedBands = 0;
    int topUsableBands = 0, extendedUsableBands = 0;

    std::vector<int> laneHistogram(width, 0);

    struct DenseComponent {
        bool valid = false;
        int startX = 0;
        int endX = 0;
        int pixels = 0;
        float centerX = 0.0f;
    };

    auto nearestComponent = [&](int beginX, int endX, float targetX) {
        DenseComponent best;
        float bestDistance = 1.0e30f;
        beginX = std::clamp(beginX, 0, width - 1);
        endX = std::clamp(endX, 0, width - 1);
        if (beginX > endX) return best;

        int x = beginX;
        while (x <= endX) {
            // No gap bridging in this pass.  A zero lane column -- including
            // the visible green/drivable separator -- terminates a component.
            while (x <= endX && laneHistogram[x] == 0) ++x;
            if (x > endX) break;

            const int runStart = x;
            long long weightedX = 0;
            int pixels = 0;
            while (x <= endX && laneHistogram[x] > 0) {
                weightedX += static_cast<long long>(x) * laneHistogram[x];
                pixels += laneHistogram[x];
                ++x;
            }
            const int runEnd = x - 1;
            if (pixels <= 0) continue;

            const float center = static_cast<float>(weightedX) / pixels;
            const float distance = std::fabs(center - targetX);
            if (!best.valid || distance < bestDistance) {
                best.valid = true;
                best.startX = runStart;
                best.endX = runEnd;
                best.pixels = pixels;
                best.centerX = center;
                bestDistance = distance;
            }
        }
        return best;
    };

    for (int y0 = scanTopY; y0 <= bottomY; y0 += denseBandHeight) {
        const int y1 = std::min(bottomY + 1, y0 + denseBandHeight);
        const float sampleY = (y0 + y1 - 1) * 0.5f;
        const float leftYellowX = m_leftTop +
            leftYellowSlope * (sampleY - static_cast<float>(topY));
        const float rightYellowX = m_rightTop +
            rightYellowSlope * (sampleY - static_cast<float>(topY));
        const float wedgeWidth = rightYellowX - leftYellowX;
        if (!std::isfinite(leftYellowX) || !std::isfinite(rightYellowX) ||
            wedgeWidth < minWedgeWidth)
            continue;

        const int roiLeft = std::clamp(
            static_cast<int>(std::floor(leftYellowX)) - edgePad, 0, width - 1);
        const int roiRight = std::clamp(
            static_cast<int>(std::ceil(rightYellowX)) + edgePad, 0, width - 1);
        if (roiLeft >= roiRight) continue;

        std::fill(laneHistogram.begin() + roiLeft,
                  laneHistogram.begin() + roiRight + 1, 0);
        for (int y = y0; y < y1; ++y) {
            const size_t row = static_cast<size_t>(y) * width;
            for (int x = roiLeft; x <= roiRight; ++x) {
                if (result.laneMask[row + x] != 0)
                    ++laneHistogram[x];
            }
        }

        // Split the wedge in two BEFORE component extraction. Therefore a
        // left-source point can never become a right-source point in one band.
        const float midF = (leftYellowX + rightYellowX) * 0.5f;
        const int leftHalfEnd = std::clamp(
            static_cast<int>(std::floor(midF)), roiLeft, roiRight);
        const int rightHalfBegin = std::clamp(
            static_cast<int>(std::ceil(midF)), roiLeft, roiRight);

        DenseComponent leftComp = nearestComponent(
            roiLeft, leftHalfEnd, leftYellowX);
        DenseComponent rightComp = nearestComponent(
            rightHalfBegin, roiRight, rightYellowX);

        // If red lane pixels are continuous through the centre split, the two
        // apparent halves are from one merged blob.  Do not create two fake
        // centres and do not connect them.  Drop the complete ambiguous band.
        bool mergedAcrossCentre = false;
        if (leftComp.valid && rightComp.valid) {
            const int centreL = std::clamp(leftHalfEnd, roiLeft, roiRight);
            const int centreR = std::clamp(rightHalfBegin, roiLeft, roiRight);
            mergedAcrossCentre = laneHistogram[centreL] > 0 &&
                                 laneHistogram[centreR] > 0 &&
                                 leftComp.endX >= centreL &&
                                 rightComp.startX <= centreR;
        }
        if (mergedAcrossCentre) {
            if (sampleY <= topY) ++topAmbiguousMergedBands;
            if (sampleY <= extendedDirectionAnchorY)
                ++extendedAmbiguousMergedBands;
            leftComp.valid = false;
            rightComp.valid = false;
        }

        if (sampleY <= topY) ++topUsableBands;
        if (sampleY <= extendedDirectionAnchorY) ++extendedUsableBands;

        if (leftComp.valid) {
            triangleLeftPoints.push_back({sampleY, leftComp.centerX});
            if (sampleY <= topY) {
                ++topLeftSourcePoints;
                topLeftSourcePixels += leftComp.pixels;
            }
            if (sampleY <= extendedDirectionAnchorY) {
                ++extendedLeftSourcePoints;
                extendedLeftSourcePixels += leftComp.pixels;
            }
        }
        if (rightComp.valid) {
            triangleRightPoints.push_back({sampleY, rightComp.centerX});
            if (sampleY <= topY) {
                ++topRightSourcePoints;
                topRightSourcePixels += rightComp.pixels;
            }
            if (sampleY <= extendedDirectionAnchorY) {
                ++extendedRightSourcePoints;
                extendedRightSourcePixels += rightComp.pixels;
            }
        }
    }

    const bool hasRedAboveYellow =
        topLeftSourcePoints + topRightSourcePoints > 0;
    const int farLeftSourcePoints = hasRedAboveYellow
        ? topLeftSourcePoints : extendedLeftSourcePoints;
    const int farRightSourcePoints = hasRedAboveYellow
        ? topRightSourcePoints : extendedRightSourcePoints;
    const long long farLeftSourcePixels = hasRedAboveYellow
        ? topLeftSourcePixels : extendedLeftSourcePixels;
    const long long farRightSourcePixels = hasRedAboveYellow
        ? topRightSourcePixels : extendedRightSourcePixels;
    const int ambiguousMergedBands = hasRedAboveYellow
        ? topAmbiguousMergedBands : extendedAmbiguousMergedBands;
    const int usableFarBands = hasRedAboveYellow
        ? topUsableBands : extendedUsableBands;

    // Replace the coarse debug centres with the dense, triangle-filtered
    // centres.  These are the only red points shown on screen.
    result.laneLeftCenterXs.clear();
    result.laneLeftCenterYs.clear();
    result.laneRightCenterXs.clear();
    result.laneRightCenterYs.clear();
    result.laneLeftCenterXs.reserve(triangleLeftPoints.size());
    result.laneLeftCenterYs.reserve(triangleLeftPoints.size());
    for (const Sample &pt : triangleLeftPoints) {
        result.laneLeftCenterXs.push_back(
            std::clamp(static_cast<int>(std::lround(pt.x)), 0, width - 1));
        result.laneLeftCenterYs.push_back(
            std::clamp(static_cast<int>(std::lround(pt.y)), 0, height - 1));
    }
    result.laneRightCenterXs.reserve(triangleRightPoints.size());
    result.laneRightCenterYs.reserve(triangleRightPoints.size());
    for (const Sample &pt : triangleRightPoints) {
        result.laneRightCenterXs.push_back(
            std::clamp(static_cast<int>(std::lround(pt.x)), 0, width - 1));
        result.laneRightCenterYs.push_back(
            std::clamp(static_cast<int>(std::lround(pt.y)), 0, height - 1));
    }

    result.roadTriangleLeftSourcePoints =
        farLeftSourcePoints;
    result.roadTriangleRightSourcePoints =
        farRightSourcePoints;

    // Direction from point SOURCE + COUNT in the forward triangle:
    //   more RIGHT-boundary points survive inside the wedge => lane bends LEFT
    //   more LEFT-boundary points survive inside the wedge  => lane bends RIGHT
    //   similar counts                                   => approximately straight
    // This intentionally avoids fitting the red centres into one curve.
    const int farTotal = farLeftSourcePoints + farRightSourcePoints;
    const long long farPixelTotal = farLeftSourcePixels + farRightSourcePixels;
    const long long farPixelDifference = farLeftSourcePixels - farRightSourcePixels;
    // Use the original amount of red evidence retained in the wedge.
    const float sourceBalance = farPixelTotal > 0
        ? static_cast<float>(farPixelDifference) /
          static_cast<float>(farPixelTotal)
        : 0.0f;
    const int expectedFarSamples = std::max(1, usableFarBands * 2);
    const float farCoverage = std::clamp(
        static_cast<float>(farTotal) / expectedFarSamples, 0.0f, 1.0f);

    // A source from the opposite side is the directional vote.  Convert pixel
    // mass to a compact band-equivalent count for display/tuning.
    const int leftTurnVotes = static_cast<int>(
        std::lround(static_cast<double>(farRightSourcePixels) /
                    std::max(1, denseBandHeight)));
    const int rightTurnVotes = static_cast<int>(
        std::lround(static_cast<double>(farLeftSourcePixels) /
                    std::max(1, denseBandHeight)));
    const int straightPairedVotes = 2 * std::min(farLeftSourcePoints,
                                                  farRightSourcePoints);
    result.roadLeftVotePoints = leftTurnVotes;
    result.roadRightVotePoints = rightTurnVotes;
    result.roadStraightVotePoints = straightPairedVotes;

    const float balanceThreshold = 0.20f;
    const int minCountDifference = 2;
    const int minFarEvidence = 4;
    const int sourceCountDifference = farLeftSourcePoints - farRightSourcePoints;
    bool directionValid = farTotal >= minFarEvidence && farCoverage >= 0.20f &&
                          farPixelTotal > 0;
    int roadCandidate = 0;
    if (directionValid) {
        if (sourceBalance <= -balanceThreshold &&
            (-sourceCountDifference >= minCountDifference ||
             leftTurnVotes >= rightTurnVotes + 2)) {
            roadCandidate = -1;
        } else if (sourceBalance >= balanceThreshold &&
                   (sourceCountDifference >= minCountDifference ||
                    rightTurnVotes >= leftTurnVotes + 2)) {
            roadCandidate = 1;
        } else {
            roadCandidate = 0;
        }
    }

    float directionConfidence = 0.0f;
    if (directionValid) {
        if (roadCandidate == 0) {
            directionConfidence = farCoverage *
                std::clamp(1.0f - std::fabs(sourceBalance), 0.0f, 1.0f);
        } else {
            directionConfidence = farCoverage *
                std::clamp(std::fabs(sourceBalance) / balanceThreshold,
                           0.0f, 1.0f);
        }
        // If many far bands were dropped because both red branches had already
        // merged into one blob, lower confidence rather than inventing a turn.
        const int denseFarBandCount = std::max(1, usableFarBands);
        const float ambiguityPenalty = std::clamp(
            1.0f - static_cast<float>(ambiguousMergedBands) /
                       (denseFarBandCount * 2.0f),
            0.45f, 1.0f);
        directionConfidence *= ambiguityPenalty;
    }

    if (directionValid) {
        if (roadCandidate == m_roadCandidateDirection) {
            ++m_roadCandidateFrames;
        } else {
            m_roadCandidateDirection = roadCandidate;
            m_roadCandidateFrames = 1;
        }
        if (m_roadCandidateFrames >= 3)
            m_roadDirection = roadCandidate;
    } else {
        m_roadCandidateDirection = 0;
        m_roadCandidateFrames = 0;
    }

    result.laneGeometryValid = true;
    result.laneGeometryTopY = topY;
    result.laneGeometryBottomY = bottomY;
    result.laneLeftTopX = std::clamp(static_cast<int>(std::lround(m_leftTop)), 0, width - 1);
    result.laneLeftBottomX = std::clamp(static_cast<int>(std::lround(m_leftBottom)), 0, width - 1);
    result.laneRightTopX = std::clamp(static_cast<int>(std::lround(m_rightTop)), 0, width - 1);
    result.laneRightBottomX = std::clamp(static_cast<int>(std::lround(m_rightBottom)), 0, width - 1);
    result.laneCenterX = std::clamp(static_cast<int>(std::lround(laneCenter)), 0, width - 1);
    result.cameraCenterX = cameraX;
    result.laneOffsetRatio = offset;
    const int usefulRows = std::max(1, (bottomY - topY) / bandHeight + 1);
    result.laneGeometryConfidence = std::clamp(
        static_cast<float>(std::min(leftLine.inliers, rightLine.inliers)) / usefulRows,
        0.0f, 1.0f);
    result.laneDepartureDirection = m_alertDirection;
    result.laneDepartureAlert = m_alertDirection != 0;
    result.roadDirection = m_roadDirection;
    // Signed source-count balance: negative = more right-source points = LEFT,
    // positive = more left-source points = RIGHT.
    result.roadDirectionRatio = sourceBalance;
    result.roadLeftCurvature = static_cast<float>(farRightSourcePoints);
    result.roadRightCurvature = static_cast<float>(farLeftSourcePoints);
    result.roadLeftCurveValid = farRightSourcePoints > 0;
    result.roadRightCurveValid = farLeftSourcePoints > 0;
    result.roadCurveTopY = scanTopY;
    result.roadLeftCurveConfidence = farCoverage;
    result.roadRightCurveConfidence = farCoverage;
    result.roadDirectionConfidence = directionValid ? directionConfidence : 0.0f;
}
