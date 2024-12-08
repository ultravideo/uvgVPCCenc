add_test(NAME template_test
  COMMAND uvgVPCCenc
  --help
)
set_tests_properties(template_test PROPERTIES TIMEOUT 120)
