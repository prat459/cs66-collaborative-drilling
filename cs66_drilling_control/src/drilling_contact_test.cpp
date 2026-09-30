#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <cmath>
#include <memory>

class DrillingContactTest : public rclcpp::Node
{
public:
    DrillingContactTest()
        : Node("drilling_contact_test"),
          current_fz_(0.0),
          baseline_fz_(0.0),
          baseline_sum_(0.0),
          baseline_samples_(0),
          calibrated_(false),
          contact_detected_(false)
    {
        // Contact must change Fz by more than this amount
        contact_threshold_ = 5.0;  // Newtons

        // Number of fresh samples used to establish the AIR baseline
        calibration_samples_ = 100;

        wrench_sub_ =
            this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                "/force_torque_sensor_broadcaster/wrench",
                10,
                std::bind(
                    &DrillingContactTest::wrenchCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "CS66 DRILLING CONTACT TEST");

        RCLCPP_INFO(
            this->get_logger(),
            "Keep the tool IN AIR and do not touch it.");

        RCLCPP_INFO(
            this->get_logger(),
            "Collecting %d samples for fresh baseline...",
            calibration_samples_);

        RCLCPP_INFO(
            this->get_logger(),
            "Contact threshold = %.2f N",
            contact_threshold_);

        RCLCPP_INFO(
            this->get_logger(),
            "========================================");
    }

private:

    void wrenchCallback(
        const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        current_fz_ = msg->wrench.force.z;

        /*
         * ---------------------------------------------------------
         * PHASE 1: CALIBRATION
         * ---------------------------------------------------------
         *
         * IMPORTANT:
         * We DO NOT check for contact during calibration.
         *
         * The previous version effectively depended on an old
         * baseline (~5.195 N). After restarting the robot/driver,
         * the zero offset can be different.
         *
         * Here we calculate a NEW baseline every time this node
         * starts.
         * ---------------------------------------------------------
         */

        if (!calibrated_)
        {
            baseline_sum_ += current_fz_;
            baseline_samples_++;

            if (baseline_samples_ % 20 == 0)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Calibrating... %d/%d   current Fz = %.3f N",
                    baseline_samples_,
                    calibration_samples_,
                    current_fz_);
            }

            if (baseline_samples_ >= calibration_samples_)
            {
                baseline_fz_ =
                    baseline_sum_ /
                    static_cast<double>(baseline_samples_);

                calibrated_ = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "========================================");

                RCLCPP_INFO(
                    this->get_logger(),
                    "CALIBRATION COMPLETE");

                RCLCPP_INFO(
                    this->get_logger(),
                    "Fresh baseline Fz = %.3f N",
                    baseline_fz_);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Contact detection is NOW ACTIVE.");

                RCLCPP_INFO(
                    this->get_logger(),
                    "Push gently on the tool to test.");

                RCLCPP_INFO(
                    this->get_logger(),
                    "========================================");
            }

            return;
        }

        /*
         * ---------------------------------------------------------
         * PHASE 2: CONTACT DETECTION
         * ---------------------------------------------------------
         *
         * Do NOT use:
         *
         *     abs(current_fz_)
         *
         * because the sensor can have gravity/tool/bias force.
         *
         * Instead:
         *
         * external force = current Fz - fresh air baseline
         * ---------------------------------------------------------
         */

        const double external_fz =
            std::abs(current_fz_ - baseline_fz_);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "Fz = %.3f N | baseline = %.3f N | external = %.3f N",
            current_fz_,
            baseline_fz_,
            external_fz);

        /*
         * Contact is triggered only once.
         */
        if (!contact_detected_ &&
            external_fz >= contact_threshold_)
        {
            contact_detected_ = true;

            RCLCPP_WARN(
                this->get_logger(),
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

            RCLCPP_WARN(
                this->get_logger(),
                "CONTACT DETECTED!");

            RCLCPP_WARN(
                this->get_logger(),
                "Fz = %.3f N",
                current_fz_);

            RCLCPP_WARN(
                this->get_logger(),
                "Baseline Fz = %.3f N",
                baseline_fz_);

            RCLCPP_WARN(
                this->get_logger(),
                "External force = %.3f N",
                external_fz);

            RCLCPP_WARN(
                this->get_logger(),
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }

        /*
         * Reset detection after the force is released.
         *
         * We use half the contact threshold as hysteresis so
         * sensor noise near 5 N does not repeatedly switch
         * CONTACT on/off.
         */
        if (contact_detected_ &&
            external_fz < (contact_threshold_ * 0.5))
        {
            contact_detected_ = false;

            RCLCPP_INFO(
                this->get_logger(),
                "Contact released. Ready for next contact.");
        }
    }

    rclcpp::Subscription<
        geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

    double current_fz_;
    double baseline_fz_;
    double baseline_sum_;

    int baseline_samples_;
    int calibration_samples_;

    double contact_threshold_;

    bool calibrated_;
    bool contact_detected_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<DrillingContactTest>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
