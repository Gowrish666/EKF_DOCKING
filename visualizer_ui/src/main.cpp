#include <QApplication>
#include <QWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QTimer>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QFont>
#include <QColor>
#include <QPointF>

#include <mqtt/async_client.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

// ============================================================
// MQTT
// ============================================================

static const std::string SERVER_ADDRESS =
    "tcp://mqtt:1883";

static const std::string CLIENT_ID =
    "visualizer";

static const std::string GROUND_TRUTH_TOPIC =
    "ekf/ground_truth";

static const std::string MEASUREMENT_TOPIC =
    "ekf/measurement";

static const std::string FILTERED_TOPIC =
    "ekf/filtered";

static const std::string RESIDUAL_TOPIC =
    "ekf/residual";

static const std::string COVARIANCE_TOPIC =
    "ekf/covariance";

static const std::string CONFIG_TOPIC =
    "ekf/config";

static const std::string STATUS_TOPIC =
    "ekf/status";

// ============================================================
// Pose
// ============================================================

struct Pose
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    bool valid = false;
};

// ============================================================
// Residual / covariance telemetry
// ============================================================

struct ResidualData
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    bool valid = false;
};

struct CovarianceData
{
    double p00 = 0.0;
    double p01 = 0.0;
    double p02 = 0.0;

    double p10 = 0.0;
    double p11 = 0.0;
    double p12 = 0.0;

    double p20 = 0.0;
    double p21 = 0.0;
    double p22 = 0.0;

    bool valid = false;
};

// ============================================================
// Shared MQTT data
// ============================================================

struct SharedData
{
    Pose groundTruth;
    Pose measurement;
    Pose filtered;

    ResidualData residual;
    CovarianceData covariance;

    std::string filterStatus = "UNKNOWN";

    unsigned long measurementSequence = 0;

    std::mutex mutex;
};

// ============================================================
// Angle helpers
// ============================================================

static double normalizeAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}

static double angleDifference(
    double target,
    double current)
{
    return normalizeAngle(target - current);
}

static double translationError(
    const Pose& groundTruth,
    const Pose& filtered)
{
    const double dx =
        filtered.x - groundTruth.x;

    const double dy =
        filtered.y - groundTruth.y;

    return std::sqrt(
        dx * dx + dy * dy
    );
}

static double rotationErrorDegrees(
    const Pose& groundTruth,
    const Pose& filtered)
{
    const double error =
        normalizeAngle(
            filtered.theta -
            groundTruth.theta
        );

    return std::abs(error) *
           180.0 /
           M_PI;
}

// ============================================================
// MQTT receiver
// ============================================================

class MqttReceiver : public mqtt::callback
{
private:

    SharedData& data_;

    bool parsePose(
        const std::string& payload,
        Pose& pose)
    {
        std::stringstream ss(payload);

        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;

        char comma1 = 0;
        char comma2 = 0;

        ss >>
            x >>
            comma1 >>
            y >>
            comma2 >>
            theta;

        if (ss.fail())
        {
            return false;
        }

        if (comma1 != ',' ||
            comma2 != ',')
        {
            return false;
        }

        pose.x = x;
        pose.y = y;
        pose.theta = theta;
        pose.valid = true;

        return true;
    }

    bool parseResidual(
        const std::string& payload,
        ResidualData& residual)
    {
        std::stringstream ss(payload);

        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;

        char comma1 = 0;
        char comma2 = 0;

        ss >>
            x >>
            comma1 >>
            y >>
            comma2 >>
            theta;

        if (ss.fail())
        {
            return false;
        }

        if (comma1 != ',' ||
            comma2 != ',')
        {
            return false;
        }

        residual.x = x;
        residual.y = y;
        residual.theta = theta;
        residual.valid = true;

        return true;
    }

    bool parseCovariance(
        const std::string& payload,
        CovarianceData& covariance)
    {
        std::stringstream ss(payload);

        double values[9] = {};

        char comma = 0;

        for (int i = 0; i < 9; ++i)
        {
            ss >> values[i];

            if (ss.fail())
            {
                return false;
            }

            if (i < 8)
            {
                ss >> comma;

                if (ss.fail() || comma != ',')
                {
                    return false;
                }
            }
        }

        covariance.p00 = values[0];
        covariance.p01 = values[1];
        covariance.p02 = values[2];

        covariance.p10 = values[3];
        covariance.p11 = values[4];
        covariance.p12 = values[5];

        covariance.p20 = values[6];
        covariance.p21 = values[7];
        covariance.p22 = values[8];

        covariance.valid = true;

        return true;
    }

public:

    explicit MqttReceiver(
        SharedData& data)
        : data_(data)
    {
    }

    void message_arrived(
        mqtt::const_message_ptr msg) override
    {
        const std::string topic =
            msg->get_topic();

        const std::string payload =
            msg->get_payload();

        std::lock_guard<std::mutex> lock(
            data_.mutex
        );

        if (topic == GROUND_TRUTH_TOPIC)
        {
            parsePose(
                payload,
                data_.groundTruth
            );
        }
        else if (topic == MEASUREMENT_TOPIC)
        {
            if (
                parsePose(
                    payload,
                    data_.measurement
                )
            )
            {
                data_.measurementSequence++;
            }
        }
        else if (topic == FILTERED_TOPIC)
        {
            parsePose(
                payload,
                data_.filtered
            );
        }
        else if (topic == RESIDUAL_TOPIC)
        {
            parseResidual(
                payload,
                data_.residual
            );
        }
        else if (topic == COVARIANCE_TOPIC)
        {
            parseCovariance(
                payload,
                data_.covariance
            );
        }
        else if (topic == STATUS_TOPIC)
        {
            data_.filterStatus = payload;
        }
    }
};

// ============================================================
// Validation state
// ============================================================

struct ValidationState
{
    static constexpr int REQUIRED_CONVERGENCE_SAMPLES = 20;

    bool converged = false;

    int consecutivePasses = 0;

    double translationSquaredSum = 0.0;

    double rotationSquaredSum = 0.0;

    unsigned long rmseSamples = 0;

    void reset()
    {
        converged = false;
        consecutivePasses = 0;

        translationSquaredSum = 0.0;
        rotationSquaredSum = 0.0;

        rmseSamples = 0;
    }

    double translationRMSE() const
    {
        if (rmseSamples == 0)
        {
            return 0.0;
        }

        return std::sqrt(
            translationSquaredSum /
            static_cast<double>(rmseSamples)
        );
    }

    double rotationRMSEDegrees() const
    {
        if (rmseSamples == 0)
        {
            return 0.0;
        }

        return std::sqrt(
            rotationSquaredSum /
            static_cast<double>(rmseSamples)
        ) *
        180.0 /
        M_PI;
    }
};

// ============================================================
// 2D Pose Visualization
// ============================================================

class PoseView : public QWidget
{
private:

    Pose groundTruth_;
    Pose measurement_;
    Pose filtered_;

    bool hasGroundTruth_ = false;
    bool hasMeasurement_ = false;
    bool hasFiltered_ = false;

    QPointF noisyTargetPosition_;
    QPointF filteredTargetPosition_;

    QPointF noisyVisualPosition_;
    QPointF filteredVisualPosition_;

    double noisyTargetTheta_ = 0.0;
    double filteredTargetTheta_ = 0.0;

    double noisyVisualTheta_ = 0.0;
    double filteredVisualTheta_ = 0.0;

    double vibrationPhase_ = 0.0;

    double noisyVibrationAmplitude_ = 1.5;

    bool animationInitialized_ = false;

    QTimer animationTimer_;

    static constexpr double NOISY_POSITION_ALPHA = 0.20;
    static constexpr double FILTERED_POSITION_ALPHA = 0.08;

    static constexpr double NOISY_THETA_ALPHA = 0.65;
    static constexpr double FILTERED_THETA_ALPHA = 0.07;

    static constexpr double NOISY_SCALE = 700.0;
    static constexpr double FILTERED_SCALE = 700.0;

    static QPointF interpolatePoint(
        const QPointF& current,
        const QPointF& target,
        double alpha)
    {
        return QPointF(
            current.x() +
                alpha *
                (target.x() - current.x()),

            current.y() +
                alpha *
                (target.y() - current.y())
        );
    }

    static double interpolateAngle(
        double current,
        double target,
        double alpha)
    {
        return normalizeAngle(
            current +
            alpha *
            angleDifference(
                target,
                current
            )
        );
    }

    QPointF localToScreen(
        const QPointF& center,
        const QPointF& local) const
    {
        return QPointF(
            center.x() + local.x(),
            center.y() - local.y()
        );
    }

    void animate()
    {
        if (!animationInitialized_)
        {
            update();
            return;
        }

        noisyVisualPosition_ =
            interpolatePoint(
                noisyVisualPosition_,
                noisyTargetPosition_,
                NOISY_POSITION_ALPHA
            );

        filteredVisualPosition_ =
            interpolatePoint(
                filteredVisualPosition_,
                filteredTargetPosition_,
                FILTERED_POSITION_ALPHA
            );

        noisyVisualTheta_ =
            interpolateAngle(
                noisyVisualTheta_,
                noisyTargetTheta_,
                NOISY_THETA_ALPHA
            );

        filteredVisualTheta_ =
            interpolateAngle(
                filteredVisualTheta_,
                filteredTargetTheta_,
                FILTERED_THETA_ALPHA
            );

        vibrationPhase_ += 0.25;

        update();
    }

    void drawGrid(
        QPainter& painter,
        const QRect& area)
    {
        painter.setPen(
            QPen(
                QColor("#243047"),
                1
            )
        );

        const int spacing = 28;

        for (
            int x = area.left() + 10;
            x < area.right();
            x += spacing)
        {
            painter.drawLine(
                x,
                area.top() + 35,
                x,
                area.bottom() - 10
            );
        }

        for (
            int y = area.top() + 35;
            y < area.bottom();
            y += spacing)
        {
            painter.drawLine(
                area.left() + 10,
                y,
                area.right() - 10,
                y
            );
        }
    }

    void drawDock(
        QPainter& painter,
        const QPointF& center)
    {
        painter.setPen(
            QPen(
                QColor("#22C55E"),
                2
            )
        );

        painter.setBrush(Qt::NoBrush);

        painter.drawEllipse(
            center,
            14,
            14
        );

        painter.drawLine(
            center.x() - 20,
            center.y(),
            center.x() + 20,
            center.y()
        );

        painter.drawLine(
            center.x(),
            center.y() - 20,
            center.x(),
            center.y() + 20
        );
    }

    void drawRobot(
        QPainter& painter,
        const QPointF& position,
        double theta,
        const QColor& color)
    {
        painter.setPen(
            QPen(
                color,
                2
            )
        );

        painter.setBrush(color);

        painter.drawEllipse(
            position,
            9,
            9
        );

        const double arrowLength = 42.0;

        const QPointF arrowEnd(
            position.x() +
                arrowLength *
                std::cos(theta),

            position.y() -
                arrowLength *
                std::sin(theta)
        );

        painter.setPen(
            QPen(
                color,
                3
            )
        );

        painter.drawLine(
            position,
            arrowEnd
        );

        const double headLength = 11.0;
        const double headAngle = 0.50;

        const QPointF left(
            arrowEnd.x() -
                headLength *
                std::cos(
                    theta - headAngle
                ),

            arrowEnd.y() +
                headLength *
                std::sin(
                    theta - headAngle
                )
        );

        const QPointF right(
            arrowEnd.x() -
                headLength *
                std::cos(
                    theta + headAngle
                ),

            arrowEnd.y() +
                headLength *
                std::sin(
                    theta + headAngle
                )
        );

        painter.drawLine(
            arrowEnd,
            left
        );

        painter.drawLine(
            arrowEnd,
            right
        );
    }

    void drawConnectionArrow(
        QPainter& painter,
        const QPointF& start,
        const QPointF& end)
    {
        painter.setPen(
            QPen(
                QColor("#CBD5E1"),
                3
            )
        );

        painter.drawLine(
            start,
            end
        );

        const double angle =
            std::atan2(
                end.y() - start.y(),
                end.x() - start.x()
            );

        const double headLength = 10.0;
        const double headAngle = 0.5;

        const QPointF left(
            end.x() -
                headLength *
                std::cos(
                    angle - headAngle
                ),

            end.y() -
                headLength *
                std::sin(
                    angle - headAngle
                )
        );

        const QPointF right(
            end.x() -
                headLength *
                std::cos(
                    angle + headAngle
                ),

            end.y() -
                headLength *
                std::sin(
                    angle + headAngle
                )
        );

        painter.drawLine(
            end,
            left
        );

        painter.drawLine(
            end,
            right
        );
    }

public:

    explicit PoseView(
        QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumSize(
            500,
            350
        );

        animationTimer_.setInterval(
            16
        );

        connect(
            &animationTimer_,
            &QTimer::timeout,
            this,
            [this]()
            {
                animate();
            }
        );

        animationTimer_.start();
    }

    void setPoses(
        const Pose& groundTruth,
        const Pose& measurement,
        const Pose& filtered)
    {
        groundTruth_ = groundTruth;
        measurement_ = measurement;
        filtered_ = filtered;

        hasGroundTruth_ =
            groundTruth.valid;

        hasMeasurement_ =
            measurement.valid;

        hasFiltered_ =
            filtered.valid;

        if (!hasGroundTruth_)
        {
            update();
            return;
        }

        if (hasMeasurement_)
        {
            double dx =
                measurement.x -
                groundTruth.x;

            double dy =
                measurement.y -
                groundTruth.y;

            dx = std::clamp(
                dx,
                -0.025,
                0.025
            );

            dy = std::clamp(
                dy,
                -0.025,
                0.025
            );

            noisyTargetPosition_ =
                QPointF(
                    dx * NOISY_SCALE,
                    dy * NOISY_SCALE
                );

            noisyTargetTheta_ =
                normalizeAngle(
                    measurement.theta
                );

            const double noiseX =
                measurement.x -
                groundTruth.x;

            const double noiseY =
                measurement.y -
                groundTruth.y;

            const double noiseMagnitude =
                std::sqrt(
                    noiseX * noiseX +
                    noiseY * noiseY
                );

            noisyVibrationAmplitude_ =
                std::clamp(
                    noiseMagnitude * 120.0,
                    1.0,
                    12.0
                );
        }

        if (hasFiltered_)
        {
            double dx =
                filtered.x -
                groundTruth.x;

            double dy =
                filtered.y -
                groundTruth.y;

            dx = std::clamp(
                dx,
                -0.05,
                0.05
            );

            dy = std::clamp(
                dy,
                -0.05,
                0.05
            );

            filteredTargetPosition_ =
                QPointF(
                    dx * FILTERED_SCALE,
                    dy * FILTERED_SCALE
                );

            filteredTargetTheta_ =
                normalizeAngle(
                    filtered.theta
                );
        }

        if (!animationInitialized_)
        {
            noisyVisualPosition_ =
                noisyTargetPosition_;

            filteredVisualPosition_ =
                filteredTargetPosition_;

            noisyVisualTheta_ =
                noisyTargetTheta_;

            filteredVisualTheta_ =
                filteredTargetTheta_;

            animationInitialized_ = true;
        }

        update();
    }

protected:

    void paintEvent(
        QPaintEvent*) override
    {
        QPainter painter(this);

        painter.setRenderHint(
            QPainter::Antialiasing
        );

        painter.fillRect(
            rect(),
            QColor("#111827")
        );

        painter.setPen(
            QColor("#F9FAFB")
        );

        painter.setFont(
            QFont(
                "Sans",
                11,
                QFont::Bold
            )
        );

        painter.drawText(
            15,
            22,
            "Noisy Pose -> EKF Filter -> Filtered Pose"
        );

        const int top = 42;
        const int bottom = height() - 12;
        const int gap = 18;

        const int leftWidth =
            static_cast<int>(
                width() * 0.36
            );

        const int middleWidth =
            static_cast<int>(
                width() * 0.18
            );

        const int rightWidth =
            static_cast<int>(
                width() * 0.36
            );

        const int totalWidth =
            leftWidth +
            middleWidth +
            rightWidth +
            2 * gap;

        const int startX =
            (width() - totalWidth) / 2;

        QRect leftArea(
            startX,
            top,
            leftWidth,
            bottom - top
        );

        QRect ekfBox(
            leftArea.right() + gap,
            top +
                (bottom - top) / 2 -
                50,
            middleWidth,
            100
        );

        QRect rightArea(
            ekfBox.right() + gap,
            top,
            rightWidth,
            bottom - top
        );

        painter.setBrush(
            QColor("#0F172A")
        );

        painter.setPen(
            QPen(
                QColor("#374151"),
                1
            )
        );

        painter.drawRoundedRect(
            leftArea,
            8,
            8
        );

        painter.drawRoundedRect(
            rightArea,
            8,
            8
        );

        painter.setBrush(
            QColor("#1F2937")
        );

        painter.setPen(
            QPen(
                QColor("#A855F7"),
                2
            )
        );

        painter.drawRoundedRect(
            ekfBox,
            8,
            8
        );

        painter.setFont(
            QFont(
                "Sans",
                11,
                QFont::Bold
            )
        );

        painter.setPen(
            QColor("#F9FAFB")
        );

        painter.drawText(
            ekfBox,
            Qt::AlignCenter,
            "EKF FILTER"
        );

        painter.setFont(
            QFont(
                "Sans",
                10,
                QFont::Bold
            )
        );

        painter.setPen(
            QColor("#EF4444")
        );

        painter.drawText(
            leftArea.left() + 10,
            leftArea.top() + 20,
            "RAW / NOISY"
        );

        painter.setPen(
            QColor("#3B82F6")
        );

        painter.drawText(
            rightArea.left() + 10,
            rightArea.top() + 20,
            "EKF FILTERED"
        );

        drawGrid(
            painter,
            leftArea
        );

        drawGrid(
            painter,
            rightArea
        );

        QPointF noisyCenter(
            leftArea.center().x(),
            leftArea.center().y() + 10
        );

        QPointF filteredCenter(
            rightArea.center().x(),
            rightArea.center().y() + 10
        );

        drawDock(
            painter,
            noisyCenter
        );

        drawDock(
            painter,
            filteredCenter
        );

        const QPointF redVibration(
            noisyVibrationAmplitude_ *
                std::sin(
                    vibrationPhase_ * 1.8
                ),

            noisyVibrationAmplitude_ *
                std::cos(
                    vibrationPhase_ * 2.4
                )
        );

        const QPointF redVisual =
            noisyVisualPosition_ +
            redVibration;

        const QPointF redScreen =
            localToScreen(
                noisyCenter,
                redVisual
            );

        const QPointF blueVibration(
            0.35 *
                std::sin(
                    vibrationPhase_ * 0.45
                ),

            0.35 *
                std::cos(
                    vibrationPhase_ * 0.40
                )
        );

        const QPointF blueVisual =
            filteredVisualPosition_ +
            blueVibration;

        const QPointF blueScreen =
            localToScreen(
                filteredCenter,
                blueVisual
            );

        if (hasMeasurement_)
        {
            drawRobot(
                painter,
                redScreen,
                noisyVisualTheta_,
                QColor("#EF4444")
            );
        }

        if (hasFiltered_)
        {
            drawRobot(
                painter,
                blueScreen,
                filteredVisualTheta_,
                QColor("#3B82F6")
            );
        }

        drawConnectionArrow(
            painter,
            QPointF(
                leftArea.right(),
                leftArea.center().y()
            ),
            QPointF(
                ekfBox.left(),
                ekfBox.center().y()
            )
        );

        drawConnectionArrow(
            painter,
            QPointF(
                ekfBox.right(),
                ekfBox.center().y()
            ),
            QPointF(
                rightArea.left(),
                rightArea.center().y()
            )
        );

        painter.setFont(
            QFont(
                "Sans",
                8
            )
        );

        if (hasMeasurement_)
        {
            painter.setPen(
                QColor("#EF4444")
            );

            painter.drawText(
                leftArea.left() + 10,
                leftArea.bottom() - 20,
                QString(
                    "x=%1  y=%2  theta=%3"
                )
                .arg(
                    measurement_.x,
                    0,
                    'f',
                    3
                )
                .arg(
                    measurement_.y,
                    0,
                    'f',
                    3
                )
                .arg(
                    measurement_.theta,
                    0,
                    'f',
                    3
                )
            );
        }

        if (hasFiltered_)
        {
            painter.setPen(
                QColor("#3B82F6")
            );

            painter.drawText(
                rightArea.left() + 10,
                rightArea.bottom() - 20,
                QString(
                    "x=%1  y=%2  theta=%3"
                )
                .arg(
                    filtered_.x,
                    0,
                    'f',
                    3
                )
                .arg(
                    filtered_.y,
                    0,
                    'f',
                    3
                )
                .arg(
                    filtered_.theta,
                    0,
                    'f',
                    3
                )
            );
        }

        if (!hasMeasurement_)
        {
            painter.setPen(
                QColor("#F59E0B")
            );

            painter.drawText(
                leftArea.left() + 10,
                leftArea.top() + 40,
                "Waiting for measurement..."
            );
        }

        if (!hasFiltered_)
        {
            painter.setPen(
                QColor("#60A5FA")
            );

            painter.drawText(
                rightArea.left() + 10,
                rightArea.top() + 40,
                "Waiting for filtered pose..."
            );
        }
    }
};

// ============================================================
// Plot Widget
// ============================================================

class PlotsWidget : public QWidget
{
private:

    static constexpr int MAX_POINTS = 250;

    std::deque<double> xRaw_;
    std::deque<double> xFiltered_;

    std::deque<double> yRaw_;
    std::deque<double> yFiltered_;

    std::deque<double> thetaRaw_;
    std::deque<double> thetaFiltered_;

    std::deque<double> noiseMagnitude_;
    std::deque<double> estimationError_;

    void appendValue(
        std::deque<double>& data,
        double value)
    {
        data.push_back(value);

        if (
            static_cast<int>(
                data.size()
            ) > MAX_POINTS
        )
        {
            data.pop_front();
        }
    }

    void drawPlot(
        QPainter& painter,
        const QRect& area,
        const QString& title,
        const std::deque<double>& first,
        const std::deque<double>& second,
        const QColor& firstColor,
        const QColor& secondColor,
        const QString& firstName,
        const QString& secondName)
    {
        painter.setPen(
            QPen(
                QColor("#374151"),
                1
            )
        );

        painter.drawRect(area);

        painter.setFont(
            QFont(
                "Sans",
                8,
                QFont::Bold
            )
        );

        painter.setPen(
            QColor("#E5E7EB")
        );

        painter.drawText(
            area.adjusted(
                6,
                2,
                -6,
                -2
            ),
            Qt::AlignTop |
            Qt::AlignLeft,
            title
        );

        if (first.empty())
        {
            return;
        }

        double minValue =
            first.front();

        double maxValue =
            first.front();

        for (double value : first)
        {
            minValue =
                std::min(
                    minValue,
                    value
                );

            maxValue =
                std::max(
                    maxValue,
                    value
                );
        }

        for (double value : second)
        {
            minValue =
                std::min(
                    minValue,
                    value
                );

            maxValue =
                std::max(
                    maxValue,
                    value
                );
        }

        if (
            std::abs(
                maxValue - minValue
            ) < 1e-9
        )
        {
            minValue -= 1.0;
            maxValue += 1.0;
        }

        const double padding =
            (maxValue - minValue) *
            0.15;

        minValue -= padding;
        maxValue += padding;

        const int left = 45;
        const int right = 10;
        const int top = 25;
        const int bottom = 15;

        QRect graphArea(
            area.left() + left,
            area.top() + top,
            area.width() - left - right,
            area.height() - top - bottom
        );

        painter.setPen(
            QPen(
                QColor("#253043"),
                1
            )
        );

        const int midY =
            graphArea.top() +
            graphArea.height() / 2;

        painter.drawLine(
            graphArea.left(),
            midY,
            graphArea.right(),
            midY
        );

        auto drawSeries =
            [&](const std::deque<double>& values,
                const QColor& color)
        {
            if (values.empty())
            {
                return;
            }

            painter.setPen(
                QPen(
                    color,
                    2
                )
            );

            QPainterPath path;

            for (
                std::size_t i = 0;
                i < values.size();
                ++i)
            {
                double normalizedX;

                if (values.size() == 1)
                {
                    normalizedX = 0.0;
                }
                else
                {
                    normalizedX =
                        static_cast<double>(i) /
                        static_cast<double>(
                            values.size() - 1
                        );
                }

                const double normalizedY =
                    (values[i] - minValue) /
                    (maxValue - minValue);

                const double px =
                    graphArea.left() +
                    normalizedX *
                    graphArea.width();

                const double py =
                    graphArea.bottom() -
                    normalizedY *
                    graphArea.height();

                if (i == 0)
                {
                    path.moveTo(
                        px,
                        py
                    );
                }
                else
                {
                    path.lineTo(
                        px,
                        py
                    );
                }
            }

            painter.drawPath(path);
        };

        drawSeries(
            first,
            firstColor
        );

        drawSeries(
            second,
            secondColor
        );

        painter.setFont(
            QFont(
                "Sans",
                7
            )
        );

        if (!firstName.isEmpty())
        {
            painter.setPen(firstColor);

            painter.drawText(
                graphArea.left(),
                graphArea.top() + 10,
                firstName
            );
        }

        if (!secondName.isEmpty())
        {
            painter.setPen(secondColor);

            painter.drawText(
                graphArea.left() + 75,
                graphArea.top() + 10,
                secondName
            );
        }
    }

public:

    explicit PlotsWidget(
        QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumSize(
            450,
            400
        );

        setAutoFillBackground(true);
    }

    void appendData(
        const Pose& groundTruth,
        const Pose& measurement,
        const Pose& filtered)
    {
        if (
            !groundTruth.valid ||
            !measurement.valid ||
            !filtered.valid
        )
        {
            return;
        }

        appendValue(
            xRaw_,
            measurement.x
        );

        appendValue(
            xFiltered_,
            filtered.x
        );

        appendValue(
            yRaw_,
            measurement.y
        );

        appendValue(
            yFiltered_,
            filtered.y
        );

        appendValue(
            thetaRaw_,
            measurement.theta
        );

        appendValue(
            thetaFiltered_,
            filtered.theta
        );

        const double noiseX =
            measurement.x -
            groundTruth.x;

        const double noiseY =
            measurement.y -
            groundTruth.y;

        const double noiseTheta =
            normalizeAngle(
                measurement.theta -
                groundTruth.theta
            );

        const double noiseMagnitude =
            std::sqrt(
                noiseX * noiseX +
                noiseY * noiseY +
                noiseTheta * noiseTheta
            );

        appendValue(
            noiseMagnitude_,
            noiseMagnitude
        );

        const double errorX =
            filtered.x -
            groundTruth.x;

        const double errorY =
            filtered.y -
            groundTruth.y;

        const double errorTheta =
            normalizeAngle(
                filtered.theta -
                groundTruth.theta
            );

        const double estimationError =
            std::sqrt(
                errorX * errorX +
                errorY * errorY +
                errorTheta * errorTheta
            );

        appendValue(
            estimationError_,
            estimationError
        );

        update();
    }

protected:

    void paintEvent(
        QPaintEvent*) override
    {
        QPainter painter(this);

        painter.setRenderHint(
            QPainter::Antialiasing
        );

        painter.fillRect(
            rect(),
            QColor("#111827")
        );

        painter.setPen(
            QColor("#E5E7EB")
        );

        painter.setFont(
            QFont(
                "Sans",
                11,
                QFont::Bold
            )
        );

        painter.drawText(
            15,
            20,
            "Real-Time EKF Plots"
        );

        const int top = 30;

        const int plotHeight =
            std::max(
                40,
                (height() - top - 10) / 5
            );

        drawPlot(
            painter,
            QRect(
                10,
                top,
                width() - 20,
                plotHeight
            ),
            "X: Raw vs Filtered",
            xRaw_,
            xFiltered_,
            QColor("#EF4444"),
            QColor("#3B82F6"),
            "Raw",
            "Filtered"
        );

        drawPlot(
            painter,
            QRect(
                10,
                top + plotHeight,
                width() - 20,
                plotHeight
            ),
            "Y: Raw vs Filtered",
            yRaw_,
            yFiltered_,
            QColor("#EF4444"),
            QColor("#3B82F6"),
            "Raw",
            "Filtered"
        );

        drawPlot(
            painter,
            QRect(
                10,
                top + 2 * plotHeight,
                width() - 20,
                plotHeight
            ),
            "Theta: Raw vs Filtered",
            thetaRaw_,
            thetaFiltered_,
            QColor("#EF4444"),
            QColor("#3B82F6"),
            "Raw",
            "Filtered"
        );

        drawPlot(
            painter,
            QRect(
                10,
                top + 3 * plotHeight,
                width() - 20,
                plotHeight
            ),
            "Sensor Noise Magnitude",
            noiseMagnitude_,
            {},
            QColor("#F59E0B"),
            QColor("#F59E0B"),
            "Noise",
            ""
        );

        drawPlot(
            painter,
            QRect(
                10,
                top + 4 * plotHeight,
                width() - 20,
                plotHeight
            ),
            "Filtered Estimation Error vs Ground Truth",
            estimationError_,
            {},
            QColor("#22C55E"),
            QColor("#22C55E"),
            "Error",
            ""
        );
    }
};

// ============================================================
// Main
// ============================================================

int main(
    int argc,
    char* argv[])
{
    QApplication app(
        argc,
        argv
    );

    // ========================================================
    // Main window
    // ========================================================

    QWidget window;

    window.setWindowTitle(
        "EKF Docking Visualiser"
    );

    window.resize(
        1400,
        900
    );

    window.setStyleSheet(
        "QWidget {"
        "    background-color: #0B1120;"
        "    color: #E5E7EB;"
        "}"
        "QGroupBox {"
        "    background-color: #111827;"
        "    border: 1px solid #374151;"
        "    border-radius: 8px;"
        "    margin-top: 18px;"
        "    padding: 8px;"
        "    font-weight: bold;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    left: 12px;"
        "    padding: 0 6px;"
        "    color: #F9FAFB;"
        "}"
        "QLabel {"
        "    color: #E5E7EB;"
        "}"
        "QSlider::groove:horizontal {"
        "    height: 6px;"
        "    background: #374151;"
        "    border-radius: 3px;"
        "}"
        "QSlider::handle:horizontal {"
        "    width: 16px;"
        "    margin: -5px 0;"
        "    border-radius: 8px;"
        "    background: #3B82F6;"
        "}"
    );

    // ========================================================
    // Shared data
    // ========================================================

    SharedData sharedData;

    MqttReceiver receiver(
        sharedData
    );

    // ========================================================
    // MQTT client
    // ========================================================

    mqtt::async_client client(
        SERVER_ADDRESS,
        CLIENT_ID
    );

    client.set_callback(
        receiver
    );

    try
    {
        client.connect()->wait();

        std::cout
            << "Visualizer connected to MQTT broker"
            << std::endl;

        client.subscribe(
            GROUND_TRUTH_TOPIC,
            1
        )->wait();

        client.subscribe(
            MEASUREMENT_TOPIC,
            1
        )->wait();

        client.subscribe(
            FILTERED_TOPIC,
            1
        )->wait();

        client.subscribe(
            RESIDUAL_TOPIC,
            1
        )->wait();

        client.subscribe(
            COVARIANCE_TOPIC,
            1
        )->wait();

        client.subscribe(
            STATUS_TOPIC,
            1
        )->wait();

        std::cout
            << "Visualizer subscribed to EKF topics"
            << std::endl;
    }
    catch (const mqtt::exception& e)
    {
        std::cerr
            << "MQTT connection error: "
            << e.what()
            << std::endl;

        return 1;
    }

    // ========================================================
    // Validation state
    // Declare this BEFORE the slider lambdas.
    // ========================================================

    ValidationState validation;

    // ========================================================
    // Root 2x2 layout
    // ========================================================

    QGridLayout* root =
        new QGridLayout(
            &window
        );

    root->setSpacing(8);

    root->setContentsMargins(
        8,
        8,
        8,
        8
    );

    root->setRowStretch(0, 1);
    root->setRowStretch(1, 1);

    root->setColumnStretch(0, 1);
    root->setColumnStretch(1, 1);

    // ========================================================
    // TOP LEFT - Pose
    // ========================================================

    QGroupBox* poseBox =
        new QGroupBox(
            "1. Noisy / Filtered Pose"
        );

    QVBoxLayout* poseLayout =
        new QVBoxLayout(
            poseBox
        );

    PoseView* poseView =
        new PoseView();

    poseLayout->addWidget(
        poseView
    );

    root->addWidget(
        poseBox,
        0,
        0
    );

    // ========================================================
    // TOP RIGHT - Plots
    // ========================================================

    QGroupBox* plotsBox =
        new QGroupBox(
            "2. Real-Time Plots"
        );

    QVBoxLayout* plotsLayout =
        new QVBoxLayout(
            plotsBox
        );

    PlotsWidget* plots =
        new PlotsWidget();

    plotsLayout->addWidget(
        plots
    );

    root->addWidget(
        plotsBox,
        0,
        1
    );

    // ========================================================
    // BOTTOM LEFT - EKF tuning
    // ========================================================

    QGroupBox* tuningBox =
        new QGroupBox(
            "3. EKF Tuning"
        );

    QVBoxLayout* tuningLayout =
        new QVBoxLayout(
            tuningBox
        );

    QLabel* qTitle =
        new QLabel(
            "Process Noise Q Scale"
        );

    QSlider* qSlider =
        new QSlider(
            Qt::Horizontal
        );

    qSlider->setRange(
        1,
        100
    );

    // Initial verification value = 0.1
    qSlider->setValue(
        10
    );

    QLabel* qValue =
        new QLabel(
            "Q Scale: 1.0"
        );

    QLabel* rTitle =
        new QLabel(
            "Measurement Noise R Scale"
        );

    QSlider* rSlider =
        new QSlider(
            Qt::Horizontal
        );

    rSlider->setRange(
        1,
        100
    );

    // Initial verification value = 1.0
    rSlider->setValue(
        10
    );

    QLabel* rValue =
        new QLabel(
            "R Scale: 1.0"
        );

    QLabel* tuningInfo =
        new QLabel(
            "Higher Q: more responsive\n"
            "Higher R: more smoothing"
        );

    tuningInfo->setWordWrap(true);

    tuningLayout->addWidget(
        qTitle
    );

    tuningLayout->addWidget(
        qSlider
    );

    tuningLayout->addWidget(
        qValue
    );

    tuningLayout->addSpacing(12);

    tuningLayout->addWidget(
        rTitle
    );

    tuningLayout->addWidget(
        rSlider
    );

    tuningLayout->addWidget(
        rValue
    );

    tuningLayout->addSpacing(12);

    tuningLayout->addWidget(
        tuningInfo
    );

    tuningLayout->addStretch();

    root->addWidget(
        tuningBox,
        1,
        0
    );

    // ========================================================
    // BOTTOM RIGHT - Validation
    // ========================================================

    QGroupBox* validationBox =
        new QGroupBox(
            "4. Validation Monitor"
        );

    QVBoxLayout* validationLayout =
        new QVBoxLayout(
            validationBox
        );

    QLabel* translationTitle =
        new QLabel(
            "Translation Error"
        );

    QLabel* translationLabel =
        new QLabel(
            "--"
        );

    QLabel* translationStatus =
        new QLabel(
            "WAITING"
        );

    translationStatus->setAlignment(
        Qt::AlignCenter
    );

    QLabel* translationLimit =
        new QLabel(
            "Limit: 0.020 m (2 cm)"
        );

    QLabel* rotationTitle =
        new QLabel(
            "Rotation Error"
        );

    QLabel* rotationLabel =
        new QLabel(
            "--"
        );

    QLabel* rotationStatus =
        new QLabel(
            "WAITING"
        );

    rotationStatus->setAlignment(
        Qt::AlignCenter
    );

    QLabel* rotationLimit =
        new QLabel(
            "Limit: 1.0 degree"
        );

    QLabel* convergenceLabel =
        new QLabel(
            "Convergence: WAITING"
        );

    QLabel* translationRMSELabel =
        new QLabel(
            "Post-Convergence Translation RMSE: --"
        );

    QLabel* rotationRMSELabel =
        new QLabel(
            "Post-Convergence Rotation RMSE: --"
        );

    QLabel* residualLabel =
        new QLabel(
            "Residual: --"
        );

    QLabel* covarianceLabel =
        new QLabel(
            "Uncertainty: --"
        );

    QLabel* filterStatusLabel =
        new QLabel(
            "Filter Status: UNKNOWN"
        );

    QLabel* statusLabel =
        new QLabel(
            "WAITING FOR DATA"
        );

    statusLabel->setAlignment(
        Qt::AlignCenter
    );

    statusLabel->setMinimumHeight(
        70
    );

    translationTitle->setStyleSheet(
        "font-weight:bold;"
    );

    rotationTitle->setStyleSheet(
        "font-weight:bold;"
    );

    translationLabel->setStyleSheet(
        "font-size:18px;font-weight:bold;"
    );

    rotationLabel->setStyleSheet(
        "font-size:18px;font-weight:bold;"
    );

    translationStatus->setStyleSheet(
        "background:#374151;"
        "color:#F9FAFB;"
        "border-radius:5px;"
        "padding:5px;"
        "font-weight:bold;"
    );

    rotationStatus->setStyleSheet(
        "background:#374151;"
        "color:#F9FAFB;"
        "border-radius:5px;"
        "padding:5px;"
        "font-weight:bold;"
    );

    convergenceLabel->setStyleSheet(
        "font-weight:bold;"
    );

    validationLayout->addWidget(
        translationTitle
    );

    validationLayout->addWidget(
        translationLabel
    );

    validationLayout->addWidget(
        translationStatus
    );

    validationLayout->addWidget(
        translationLimit
    );

    validationLayout->addSpacing(6);

    validationLayout->addWidget(
        rotationTitle
    );

    validationLayout->addWidget(
        rotationLabel
    );

    validationLayout->addWidget(
        rotationStatus
    );

    validationLayout->addWidget(
        rotationLimit
    );

    validationLayout->addSpacing(8);

    validationLayout->addWidget(
        convergenceLabel
    );

    validationLayout->addWidget(
        translationRMSELabel
    );

    validationLayout->addWidget(
        rotationRMSELabel
    );

    validationLayout->addWidget(
        residualLabel
    );

    validationLayout->addWidget(
        covarianceLabel
    );

    validationLayout->addWidget(
        filterStatusLabel
    );

    validationLayout->addStretch();

    validationLayout->addWidget(
        statusLabel
    );

    root->addWidget(
        validationBox,
        1,
        1
    );

    // ========================================================
    // Q/R publishing
    // ========================================================

    auto publishConfig =
        [&]()
        {
            const double qScale =
                static_cast<double>(
                    qSlider->value()
                ) / 10.0;

            const double rScale =
                static_cast<double>(
                    rSlider->value()
                ) / 10.0;

            const std::string payload =
                std::to_string(qScale) +
                "," +
                std::to_string(rScale);

            auto message =
                mqtt::make_message(
                    CONFIG_TOPIC,
                    payload
                );

            message->set_qos(1);

            try
            {
                client.publish(
                    message
                );
            }
            catch (const mqtt::exception& e)
            {
                std::cerr
                    << "Config publish failed: "
                    << e.what()
                    << std::endl;
            }
        };

    // ========================================================
    // Q slider
    // ========================================================

    QObject::connect(
        qSlider,
        &QSlider::valueChanged,
        [&](int value)
        {
            const double scale =
                static_cast<double>(value) /
                10.0;

            qValue->setText(
                QString(
                    "Q Scale: %1"
                )
                .arg(
                    scale,
                    0,
                    'f',
                    1
                )
            );

            validation.reset();

            translationStatus->setText(
                "WAITING"
            );

            rotationStatus->setText(
                "WAITING"
            );

            statusLabel->setText(
                "WAITING FOR CONVERGENCE"
            );

            convergenceLabel->setText(
                "Convergence: WAITING"
            );

            translationRMSELabel->setText(
                "Post-Convergence Translation RMSE: --"
            );

            rotationRMSELabel->setText(
                "Post-Convergence Rotation RMSE: --"
            );

            publishConfig();
        }
    );

    // ========================================================
    // R slider
    // ========================================================

    QObject::connect(
        rSlider,
        &QSlider::valueChanged,
        [&](int value)
        {
            const double scale =
                static_cast<double>(value) /
                10.0;

            rValue->setText(
                QString(
                    "R Scale: %1"
                )
                .arg(
                    scale,
                    0,
                    'f',
                    1
                )
            );

            validation.reset();

            translationStatus->setText(
                "WAITING"
            );

            rotationStatus->setText(
                "WAITING"
            );

            statusLabel->setText(
                "WAITING FOR CONVERGENCE"
            );

            convergenceLabel->setText(
                "Convergence: WAITING"
            );

            translationRMSELabel->setText(
                "Post-Convergence Translation RMSE: --"
            );

            rotationRMSELabel->setText(
                "Post-Convergence Rotation RMSE: --"
            );

            publishConfig();
        }
    );

    // ========================================================
    // Live update timer
    // ========================================================

    QTimer* timer =
        new QTimer(
            &window
        );

    unsigned long lastSequence = 0;

    QObject::connect(
        timer,
        &QTimer::timeout,
        [&]()
        {
            Pose groundTruth;
            Pose measurement;
            Pose filtered;

            ResidualData residual;
            CovarianceData covariance;

            std::string filterStatus;

            unsigned long sequence = 0;

            {
                std::lock_guard<std::mutex> lock(
                    sharedData.mutex
                );

                groundTruth =
                    sharedData.groundTruth;

                measurement =
                    sharedData.measurement;

                filtered =
                    sharedData.filtered;

                residual =
                    sharedData.residual;

                covariance =
                    sharedData.covariance;

                filterStatus =
                    sharedData.filterStatus;

                sequence =
                    sharedData.measurementSequence;
            }

            const bool newMeasurement =
                sequence != lastSequence;

            // ------------------------------------------------
            // Top-left pose display
            // ------------------------------------------------

            poseView->setPoses(
                groundTruth,
                measurement,
                filtered
            );

            // ------------------------------------------------
            // Plots
            // ------------------------------------------------

            if (
                newMeasurement &&
                groundTruth.valid &&
                measurement.valid &&
                filtered.valid
            )
            {
                plots->appendData(
                    groundTruth,
                    measurement,
                    filtered
                );
            }

            // ------------------------------------------------
            // Validation
            // IMPORTANT:
            // only accumulate once per new measurement
            // ------------------------------------------------

            if (
                newMeasurement &&
                groundTruth.valid &&
                filtered.valid
            )
            {
                const double tError =
                    translationError(
                        groundTruth,
                        filtered
                    );

                const double rError =
                    rotationErrorDegrees(
                        groundTruth,
                        filtered
                    );

                translationLabel->setText(
                    QString(
                        "%1 m"
                    )
                    .arg(
                        tError,
                        0,
                        'f',
                        4
                    )
                );

                rotationLabel->setText(
                    QString(
                        "%1 deg"
                    )
                    .arg(
                        rError,
                        0,
                        'f',
                        3
                    )
                );

                const bool translationPass =
                    tError <= 0.020;

                const bool rotationPass =
                    rError <= 1.0;

                if (translationPass)
                {
                    translationStatus->setText(
                        "PASS"
                    );

                    translationStatus->setStyleSheet(
                        "background:#166534;"
                        "color:#DCFCE7;"
                        "border-radius:5px;"
                        "padding:5px;"
                        "font-weight:bold;"
                    );
                }
                else
                {
                    translationStatus->setText(
                        "FAIL"
                    );

                    translationStatus->setStyleSheet(
                        "background:#991B1B;"
                        "color:#FEE2E2;"
                        "border-radius:5px;"
                        "padding:5px;"
                        "font-weight:bold;"
                    );
                }

                if (rotationPass)
                {
                    rotationStatus->setText(
                        "PASS"
                    );

                    rotationStatus->setStyleSheet(
                        "background:#166534;"
                        "color:#DCFCE7;"
                        "border-radius:5px;"
                        "padding:5px;"
                        "font-weight:bold;"
                    );
                }
                else
                {
                    rotationStatus->setText(
                        "FAIL"
                    );

                    rotationStatus->setStyleSheet(
                        "background:#991B1B;"
                        "color:#FEE2E2;"
                        "border-radius:5px;"
                        "padding:5px;"
                        "font-weight:bold;"
                    );
                }

                // ------------------------------------------------
                // Convergence
                // ------------------------------------------------

                if (!validation.converged)
                {
                    if (
                        translationPass &&
                        rotationPass
                    )
                    {
                        validation.consecutivePasses++;
                    }
                    else
                    {
                        validation.consecutivePasses = 0;
                    }

                    if (
                        validation.consecutivePasses >=
                        ValidationState::
                        REQUIRED_CONVERGENCE_SAMPLES
                    )
                    {
                        validation.converged = true;
                    }
                }

                if (validation.converged)
                {
                    convergenceLabel->setText(
                        "Convergence: CONVERGED"
                    );

                    // --------------------------------------------
                    // Post-convergence RMSE
                    // --------------------------------------------

                    const double dx =
                        filtered.x -
                        groundTruth.x;

                    const double dy =
                        filtered.y -
                        groundTruth.y;

                    const double dTheta =
                        normalizeAngle(
                            filtered.theta -
                            groundTruth.theta
                        );

                    validation.translationSquaredSum +=
                        dx * dx +
                        dy * dy;

                    validation.rotationSquaredSum +=
                        dTheta * dTheta;

                    validation.rmseSamples++;

                    const double translationRMSE =
                        validation.translationRMSE();

                    const double rotationRMSE =
                        validation.rotationRMSEDegrees();

                    translationRMSELabel->setText(
                        QString(
                            "Post-Convergence Translation RMSE: %1 m"
                        )
                        .arg(
                            translationRMSE,
                            0,
                            'f',
                            4
                        )
                    );

                    rotationRMSELabel->setText(
                        QString(
                            "Post-Convergence Rotation RMSE: %1 deg"
                        )
                        .arg(
                            rotationRMSE,
                            0,
                            'f',
                            3
                        )
                    );

                    const bool rmseTranslationPass =
                        translationRMSE <= 0.020;

                    const bool rmseRotationPass =
                        rotationRMSE <= 1.0;

                    if (
                        rmseTranslationPass &&
                        rmseRotationPass
                    )
                    {
                        statusLabel->setText(
                            "SAFE TO DOCK"
                        );

                        statusLabel->setStyleSheet(
                            "background:#166534;"
                            "color:#DCFCE7;"
                            "border-radius:8px;"
                            "font-size:22px;"
                            "font-weight:bold;"
                            "padding:8px;"
                        );
                    }
                    else
                    {
                        statusLabel->setText(
                            "FAIL"
                        );

                        statusLabel->setStyleSheet(
                            "background:#991B1B;"
                            "color:#FEE2E2;"
                            "border-radius:8px;"
                            "font-size:22px;"
                            "font-weight:bold;"
                            "padding:8px;"
                        );
                    }
                }
                else
                {
                    convergenceLabel->setText(
                        QString(
                            "Convergence: %1 / %2"
                        )
                        .arg(
                            validation.consecutivePasses
                        )
                        .arg(
                            ValidationState::
                            REQUIRED_CONVERGENCE_SAMPLES
                        )
                    );

                    statusLabel->setText(
                        "WAITING FOR CONVERGENCE"
                    );

                    statusLabel->setStyleSheet(
                        "background:#374151;"
                        "color:#F9FAFB;"
                        "border-radius:8px;"
                        "font-size:20px;"
                        "font-weight:bold;"
                        "padding:8px;"
                    );
                }
            }

            // ------------------------------------------------
            // Residual telemetry
            // ------------------------------------------------

            if (residual.valid)
            {
                residualLabel->setText(
                    QString(
                        "Residual: x=%1 y=%2 theta=%3"
                    )
                    .arg(
                        residual.x,
                        0,
                        'f',
                        4
                    )
                    .arg(
                        residual.y,
                        0,
                        'f',
                        4
                    )
                    .arg(
                        residual.theta,
                        0,
                        'f',
                        4
                    )
                );
            }

            // ------------------------------------------------
            // Covariance telemetry
            // ------------------------------------------------

            if (covariance.valid)
            {
                const double sigmaX =
                    std::sqrt(
                        std::max(
                            0.0,
                            covariance.p00
                        )
                    );

                const double sigmaY =
                    std::sqrt(
                        std::max(
                            0.0,
                            covariance.p11
                        )
                    );

                const double sigmaThetaDeg =
                    std::sqrt(
                        std::max(
                            0.0,
                            covariance.p22
                        )
                    ) *
                    180.0 /
                    M_PI;

                covarianceLabel->setText(
                    QString(
                        "Uncertainty: sigmaX=%1  sigmaY=%2  sigmaTheta=%3 deg"
                    )
                    .arg(
                        sigmaX,
                        0,
                        'f',
                        4
                    )
                    .arg(
                        sigmaY,
                        0,
                        'f',
                        4
                    )
                    .arg(
                        sigmaThetaDeg,
                        0,
                        'f',
                        3
                    )
                );
            }

            // ------------------------------------------------
            // EKF status
            // ------------------------------------------------

            if (!filterStatus.empty())
            {
                filterStatusLabel->setText(
                    QString(
                        "Filter Status: %1"
                    )
                    .arg(
                        QString::fromStdString(
                            filterStatus
                        )
                    )
                );
            }

            if (newMeasurement)
            {
                lastSequence = sequence;
            }
        }
    );

    timer->start(16);

    // Send initial Q/R configuration
    publishConfig();

    // ========================================================
    // Show
    // ========================================================

    window.show();

    return app.exec();
}