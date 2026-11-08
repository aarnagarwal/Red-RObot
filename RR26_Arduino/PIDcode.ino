/*
struct {
    float base_speed;
    float gain;
    //consider error
} PID_info;

/*
okay so what do we need? we need
struct

be able to make new struct
able to edit struct
able to read struct

also want to calculate the stuff helpers? such as error*gain stuff?
could also have a helper for calculating what to fix the err field or smth to
*/

/*
PID_info new_PID(float base_speed, float gain) {
    PID_info new = malloc(sizeof(PID_info));
    new->base_speed = malloc(sizeof(float));
    new->gain = malloc(sizeof(float));

    new->base_speed = base_speed;
    new->gain = gain;
    return new;
}

void edit_PID(PID_info PID, float base_speed, float gain) {
    PID->base_speed = base_speed;
    PID->gain = gain;
}
*/