package VERTEX_Simulations;

import io.gatling.javaapi.core.*;
import io.gatling.javaapi.http.*;

import static io.gatling.javaapi.core.CoreDsl.*;
import static io.gatling.javaapi.http.HttpDsl.*;

public class SoakTestSimulation extends Simulation {

    HttpProtocolBuilder httpProtocol = http
            .baseUrl("http://localhost:5000")
            .userAgentHeader("Mozilla/5.0");

    ScenarioBuilder scn = scenario("Soak test")
            .exec(
                    http("Open homepage")
                            .get("/")
                            .check(status().is(200))
            )
            .pause(1);

    {
        setUp(
                scn.injectOpen(constantUsersPerSec(5).during(300))
        ).protocols(httpProtocol);
    }
}