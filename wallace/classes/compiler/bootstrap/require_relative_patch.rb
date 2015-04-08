# Enables Ruby 1.8.7 support.
# http://stackoverflow.com/questions/4333286/ruby-require-vs-require-relative-best-practice-to-workaround-running-in-both
unless Kernel.respond_to?(:require_relative)
  module Kernel
    def require_relative(path)
      require File.join(File.dirname(caller[0]), path.to_str)
    end
  end
end
