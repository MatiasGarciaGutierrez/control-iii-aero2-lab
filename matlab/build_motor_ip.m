function model = build_motor_ip
%BUILD_MOTOR_IP Build the simple question model and its experiment copy.
experiment = build_motor_ip_experiment;
model = create_question_model(experiment, 2);
end
