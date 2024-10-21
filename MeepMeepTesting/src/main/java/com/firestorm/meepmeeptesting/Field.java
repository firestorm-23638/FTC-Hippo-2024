package com.firestorm.meepmeeptesting;

import com.acmerobotics.roadrunner.Vector2d;

public class Field {

    public static final Vector2d RED_BASKET = new Vector2d(-56.1923881554, -55.5502525317);
    public static final Vector2d[] RED_BASKET_YELLOW_SAMPLES = {
            new Vector2d(-48, -25.5),
            new Vector2d(-57, -25.5),
            new Vector2d(-67, -25.5)
    };
    public static final Vector2d RED_OBSERVATION = new Vector2d(46.9, -61);

    public static final Vector2d BLUE_BASKET = new Vector2d(56.1923881554, 55.5502525317);
    public static final Vector2d[] BLUE_BASKET_YELLOW_SAMPLES = {
            new Vector2d(48, 25.5),
            new Vector2d(57, 25.5),
            new Vector2d(67, 25.5)
    };
    public static final Vector2d BLUE_OBSERVATION = new Vector2d(46.9, 61);
}
