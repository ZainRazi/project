# encoding: utf-8

Wallace::Boolean = Object.new
Wallace::Boolean.send(:define_singleton_method, :new, &lambda{ |v| v })
