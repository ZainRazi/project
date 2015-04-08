# encoding: utf-8
Wallace::Wildcard = Object.new
Wallace::Wildcard.send(:define_singleton_method, :new, &lambda{ |v| v })
