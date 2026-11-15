function model = build_motor_pi
%BUILD_MOTOR_PI Build the simple question model and its experiment copy.
experiment = build_motor_pi_experiment;
model = create_question_model(experiment, 1);
end
