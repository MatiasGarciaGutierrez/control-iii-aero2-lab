function model = build_feedback_linearization
%BUILD_FEEDBACK_LINEARIZATION Build the simple question model and its experiment copy.
experiment = build_feedback_linearization_experiment;
model = create_question_model(experiment, 3);
end
