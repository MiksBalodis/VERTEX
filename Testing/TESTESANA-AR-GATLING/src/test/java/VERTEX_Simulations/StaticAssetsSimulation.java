package VERTEX_Simulations;

import io.gatling.javaapi.core.*;
import io.gatling.javaapi.http.*;

import static io.gatling.javaapi.core.CoreDsl.*;
import static io.gatling.javaapi.http.HttpDsl.*;

public class StaticAssetsSimulation extends Simulation {

    HttpProtocolBuilder httpProtocol = http
            .baseUrl("http://localhost:5000")
            .userAgentHeader("Mozilla/5.0");

    ScenarioBuilder scn = scenario("Static assets test")
            .exec(
                    http("Open homepage")
                            .get("/")
                            .check(status().is(200))
            )
            .exec(
                    http("Load CSS")
                            .get("/global.css")
                            .check(status().is(200))
            )
            .exec(
                    http("Load favicon")
                            .get("/favicon.png")
                            .check(status().is(200))
            );

    {
        setUp(
                scn.injectOpen(rampUsers(50).during(30))
        ).protocols(httpProtocol);
    }
}