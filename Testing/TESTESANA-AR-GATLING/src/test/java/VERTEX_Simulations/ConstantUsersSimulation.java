package VERTEX_Simulations;

import io.gatling.javaapi.core.*;
import io.gatling.javaapi.http.*;

import static io.gatling.javaapi.core.CoreDsl.*;
import static io.gatling.javaapi.http.HttpDsl.*;

public class ConstantUsersSimulation extends Simulation {

    HttpProtocolBuilder httpProtocol = http
            .baseUrl("http://localhost:5000")
            .userAgentHeader("Mozilla/5.0");

    ScenarioBuilder scn = scenario("Constant users load test")
            .exec(
                    http("Open homepage")
                            .get("/")
                            .check(status().is(200))
            );

    {
        setUp(
                scn.injectOpen(constantUsersPerSec(10).during(30))
        ).protocols(httpProtocol);
    }
}