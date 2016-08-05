#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Vector.h>

namespace gtsam {

    class Switch: public DerivedValue<Switch> {
    protected:
        /* Required by GenericValue.equals_ */
        Vector1 v_;

    public:
        Switch(double d) {
            v_ << d;
        }

        Vector1 val() const {
            return v_;
        }

        /* Required for Testable trait */
        virtual void print(const std::string &str="") const {
        }

        /* Required for Testable trait */
        bool equals(const Switch &other, double tol = 1e-9) const {
            return false;
        }

        virtual size_t dim() const {
            return 1;
        }

        /* Required for DerivedValue.retract_ */
        Switch retract(const Vector & delta) const {
            return Switch(max(min(v_[0] + delta[0], 1.0), 0.0);
        }

        /* Required for DerivedValue.localCoordinates_ */
        Vector localCoordinates(const Switch &s) const {
            return s.val() - v_;
        }

    };

    template<>
    struct traits<Switch>: Testable<Switch> {
        /** Required b/c of GenericValue **/
        static Vector1 Local(Switch origin, Switch other,
                             OptionalJacobian<1,1> H1 = boost::none,
                             OptionalJacobian<1,1> H2 = boost::none) {
            if (H1) (*H1)[0] = -1.0;
            if (H2) (*H2)[0] =  1.0;
            return origin.localCoordinates(other);
        }

        /** Required b/c of GenericValue **/
        static Switch Retract(Switch origin, Vector1 & v,
                              OptionalJacobian<1,1> H1 = boost::none,
                              OptionalJacobian<1,1> H2 = boost::none) {
            if (H1) (*H1)[0] = 1.0;
            if (H2) (*H2)[0] = 1.0;
            return origin.retract(v);
        }

        /** Required b/c of GenericValue **/
        static int GetDimension(const Switch& s) {
            return s.dim();
        }
    };

    template<class VALUE>
    class BetweenFactorHai: public NoiseModelFactor3<VALUE, VALUE, Switch> {
        public:
        gtsam::BetweenFactor<VALUE> betweenFactor;

        /** default constructor - only use for serialization */
        BetweenFactorHai() {}

        /** Constructor */
        BetweenFactorHai(Key key1, Key key2, Key key3, const VALUE& measured,
            const SharedNoiseModel& model) :
            NoiseModelFactor3<VALUE, VALUE, Switch>(model, key1, key2, key3),
            betweenFactor(key1, key2, measured, model)
        {

        }

        Vector evaluateError(const VALUE& p1, const VALUE& p2, const Switch& s,
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 =  boost::none,
          boost::optional<gtsam::Matrix&> H3 =  boost::none) const
        {
            Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);
            error *= s.val()[0];
            if (H1) *H1 = *H1 * s.val()[0];
            if (H2) *H2 = *H2 * s.val()[0];
            if (H3) *H3 = error;

            return error;
        }

        virtual ~BetweenFactorHai() {}
    };

    template<class VALUE>
        struct traits<BetweenFactorHai<VALUE> > : public Testable<BetweenFactorHai<VALUE> > {};
}
