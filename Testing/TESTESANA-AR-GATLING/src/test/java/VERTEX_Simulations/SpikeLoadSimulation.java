package VERTEX_Simulations;

import io.gatling.javaapi.core.*;
import io.gatling.javaapi.http.*;

import static io.gatling.javaapi.core.CoreDsl.*;
import static io.gatling.javaapi.http.HttpDsl.*;

public class SpikeLoadSimulation extends Simulation {

    HttpProtocolBuilder httpProtocol = http
            .baseUrl("http://localhost:5000")
            .userAgentHeader("Mozilla/5.0");

    ScenarioBuilder scn = scenario("Spike load test")
            .exec(
                    http("Open homepage")
                            .get("/")
                            .check(status().is(200))
            );

    {
        setUp(
                scn.injectOpen(atOnceUsers(200))
        ).protocols(httpProtocol);
    }
}