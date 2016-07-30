#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Vector.h>

namespace gtsam {
    template<class VALUE>
    class BetweenFactorHai: public NoiseModelFactor3<VALUE, VALUE, Vector1> {
        public:
        gtsam::BetweenFactor<VALUE> betweenFactor;

        /** default constructor - only use for serialization */
        BetweenFactorHai() {}

        /** Constructor */
        BetweenFactorHai(Key key1, Key key2, Key key3, const VALUE& measured,
            const SharedNoiseModel& model) :
            NoiseModelFactor3<VALUE, VALUE, Vector1>(model, key1, key2, key3),
            betweenFactor(key1, key2, measured, model)
        {

        }

        Vector evaluateError(const VALUE& p1, const VALUE& p2, const Vector1& s,
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 =  boost::none,
          boost::optional<gtsam::Matrix&> H3 =  boost::none) const
        {
            Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);
            error *= s.value();
            if (H1) *H1 = *H1 * s.value();
            if (H2) *H2 = *H2 * s.value();
            if (H3) *H3 = error;

            return error;
        }

        virtual ~BetweenFactorHai() {}
    };

    template<class VALUE>
        struct traits<BetweenFactorHai<VALUE> > : public Testable<BetweenFactorHai<VALUE> > {};
}
